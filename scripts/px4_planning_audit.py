#!/usr/bin/env python3
"""Read-only real-map planning graph audit; never submits a goal or FMU input."""
import json
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from uav_nav_interfaces.msg import PlanningContext, MapSnapshot, LocalizedOdometry
from uav_nav_sim.core import grid_from_message
from uav_mission.ego_execution import BrakingGrid


class Audit(Node):
    def __init__(self, evidence, session):
        super().__init__('px4_depth_planning_audit')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.evidence, self.session = evidence, session
        self.map = self.odom = None
        self.contexts = self.valid_contexts = 0
        self.create_subscription(MapSnapshot, '/planning/source/map', self.map_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LocalizedOdometry, '/uav/px4/localized_odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PlanningContext, '/planning/context', self.context_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def map_cb(self, message):self.map = (message, time.monotonic())
    def odom_cb(self, message):self.odom = (message, time.monotonic())

    def context_cb(self, message):
        self.contexts += 1
        if message.valid:self.valid_contexts += 1
        writers = {topic: len(self.get_publishers_info_by_topic(topic)) for topic in
                   ('/planning/source/map','/uav/px4/localized_odometry','/planning/source/alignment',
                    '/planning/context','/planning/bound_trajectory','/planning/ego/trajectory')}
        readers = self.count_subscribers('/planning/ego/goal')
        matched = False
        clear = None
        if self.map and self.odom:
            m, mt = self.map; o, ot = self.odom
            matched = ((message.map_id,message.map_epoch)==(m.map_id,m.epoch)
                       and message.localization_session == o.localization_session == self.session
                       and tuple(message.reset_counters)==tuple(o.reset_counters)
                       and message.alignment_generation==1
                       and message.alignment_id=='depth-map-is-ekf-odom-v1'
                       and time.monotonic()-mt<=2. and time.monotonic()-ot<=.5)
            if m.valid and message.valid:
                p = o.odometry.pose.pose.position
                # Explicit identity map <- odom in this disarmed reference.
                grid = BrakingGrid(grid_from_message(m), 1.2)
                clear = not grid.collision([p.x,p.y,p.z], .8)
        passed = (message.valid and matched and self.valid_contexts>=5 and readers==1
                  and all(value==1 for topic,value in writers.items() if topic!='/planning/ego/trajectory')
                  and writers['/planning/ego/trajectory']==1)
        data = dict(passed=bool(passed), contexts=self.contexts, valid_contexts=self.valid_contexts,
                    reason=message.reason, context_id=message.context_id,
                    map_id=message.map_id, map_epoch=message.map_epoch, map_version=message.map_version,
                    localization_session=message.localization_session, reset_counters=[int(v) for v in message.reset_counters],
                    alignment_id=message.alignment_id, alignment_generation=message.alignment_generation,
                    writers=writers, planner_goal_readers=readers,
                    flight_envelope_start_clear=clear, goal_dispatched=False,
                    captured_monotonic=time.monotonic(),
                    scope='actual online depth-map and PX4 EKF planning context; no trajectory or flight qualification')
        temporary = self.evidence.with_suffix('.tmp')
        temporary.write_text(json.dumps(data,indent=2)+'\n');temporary.replace(self.evidence)


if __name__=='__main__':
    rclpy.init(args=[]);node=Audit(Path(sys.argv[1]),sys.argv[2])
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
