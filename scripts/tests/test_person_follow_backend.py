"""Real ROS selection/services/status integration, with synthetic sensor inputs only."""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import uuid
from copy import deepcopy

import pytest
pytest.importorskip('rclpy')
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from uav_nav_interfaces.msg import PersonObservation,PersonObservationArray,PlanningContext,TaskStatus
from visualization_msgs.msg import MarkerArray
from uav_nav_interfaces.srv import SetMissionIntent
from vision_msgs.msg import Detection2DArray,Detection2D

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from person_follow_backend import PersonFollowBackend
from person_follow_client import PersonFollowClient
from vision_overlay import TargetSelection
from mission_person_observation import EpisodeConfig


def test_selected_rgb_binds_3d_preview_services_and_faults(tmp_path):
    rclpy.init()
    source = rclpy.create_node('person_follow_test_source')
    backend = PersonFollowBackend(tmp_path/'person-motion.json')
    ui = PersonFollowClient(source)
    goal_messages=[]
    trajectory_messages=[]
    source.create_subscription(PoseStamped,'/uav/people/follow/preview_goal',goal_messages.append,10)
    source.create_subscription(MarkerArray,'/uav/people/motion_trajectories',trajectory_messages.append,10)
    try:
        publishers={key:source.create_publisher(kind,topic,10) for key,kind,topic in (
            ('clock',Clock,'/clock'),('context',PlanningContext,'/planning/context'),
            ('info',CameraInfo,'/px4_depth/camera_info'),('odom',Odometry,'/planning/ego/odometry'),
            ('observations',PersonObservationArray,'/uav/people/observations'),
            ('detections',Detection2DArray,'/uav/people/detections'),('task',TaskStatus,'/uav/bt/task_status'))}
        tf=StaticTransformBroadcaster(source)
        transform=TransformStamped()
        transform.header.frame_id,transform.child_frame_id='map','camera'
        transform.transform.rotation.w=1.
        tf.sendTransform(transform)
        # The UI node uses system time by default; give it the same sim clock as the monitor.
        source.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        clock=Clock()
        clock.clock.sec=10
        stamp=deepcopy(clock.clock)
        context=PlanningContext()
        context.header.frame_id='map'
        context.header.stamp=stamp
        context.map_id,context.map_epoch='map-a',1
        context.localization_session,context.alignment_id,context.context_id='session','align','context'
        context.alignment_generation=1
        context.valid=True
        identity=['map-a',1,'session','align',1,'context']
        info=CameraInfo()
        info.header.frame_id='camera'
        info.header.stamp=stamp
        info.width=info.height=200
        info.k=[100.,0.,100.,0.,100.,100.,0.,0.,1.]
        odom=Odometry()
        odom.header=context.header
        odom.child_frame_id='base_link'
        odom.pose.pose.position.x=-5.
        odom.pose.pose.position.z=1.
        odom.pose.pose.orientation.w=1.
        observations=PersonObservationArray()
        observations.header=context.header
        observations.map_id,observations.map_epoch='map-a',1
        observations.localization_session,observations.alignment_id,observations.context_id='session','align','context'
        observations.alignment_generation=1
        observations.valid=True
        observations.point_semantics='visible_torso_surface'
        person=PersonObservation(track_id='1:person',detection_confidence=.9,depth_spread_m=.05,depth_samples=30)
        person.position.z=2.
        person.last_observed=stamp
        observations.observations=[person]
        detections=Detection2DArray()
        detections.header=info.header
        box=Detection2D()
        box.bbox.center.position.x=box.bbox.center.position.y=100.
        box.bbox.size_x=box.bbox.size_y=60.
        detections.detections=[box]
        task=TaskStatus(phase='NAVIGATE',coordinator_instance='test-coordinator',total_remaining_s=100.)
        task.header=context.header
        task.mission_uuid.uuid=list(uuid.uuid4().bytes)
        task.control_session.session_id.uuid=list(uuid.uuid4().bytes)
        task.control_session.generation=1
        task.control_session.owner='TASK'
        selection=TargetSelection()
        selection.choose([(70,70,130,130)],0,('camera',10.),time.monotonic())

        capture=10.
        frozen=False
        last_publish=0.
        moving=False
        motion_origin=10.
        def pump(seconds,heartbeat=True):
            nonlocal capture,last_publish
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                total_ns=clock.clock.sec*1000000000+clock.clock.nanosec+1000000
                clock.clock.sec,clock.clock.nanosec=divmod(total_ns,1000000000)
                publishers['clock'].publish(clock)
                if time.monotonic()-last_publish>.04:
                    last_publish=time.monotonic()
                    if not frozen:
                        capture=clock.clock.sec+clock.clock.nanosec*1e-9
                        for msg in (context,info,odom,observations,detections,task):
                            msg.header.stamp=deepcopy(clock.clock)
                        if moving:
                            person.position.x=.4*(capture-motion_origin)
                        person.last_observed=deepcopy(clock.clock)
                        observations.observations=[person]
                    for key,msg in [('context',context),('info',info),('odom',odom),('observations',observations),('detections',detections),('task',task)]:
                        publishers[key].publish(msg)
                if heartbeat:
                    ui.send_selection(selection,('camera',capture),identity)
                for _ in range(12):
                    rclpy.spin_once(source,timeout_sec=0)
                    rclpy.spin_once(backend,timeout_sec=0)
                ui.poll()
                time.sleep(.002)

        pump(.35)
        assert backend.core.state=='READY',backend.core.reason
        status=ui.current(selection.sequence)
        assert status is not None and status['track_id']=='1:person'
        assert ui.command('START',selection.sequence)[0]
        pump(.12)
        assert backend.core.state=='PREVIEW'
        assert backend.core.goal==[-4.5,0.,1.]
        assert goal_messages
        assert 'PREVIEW_STARTED' in ui.notice
        assert ui.command('PAUSE',selection.sequence)[0]
        pump(.1)
        assert backend.core.state=='PAUSED' and backend.core.goal is None
        assert ui.command('RESUME',selection.sequence)[0]
        pump(.1)
        assert backend.core.state=='PREVIEW'
        assert ui.command('EXECUTE',selection.sequence)[0]
        pump(.1)
        assert 'DYNAMIC_SAFETY_UNQUALIFIED' in ui.notice
        assert ui.command('CANCEL',selection.sequence)[0]
        pump(.1)
        assert not backend.core.active and backend.core.goal is None
        own=backend.get_publisher_names_and_types_by_node(backend.get_name(),'/')
        assert {topic for topic,_ in own} <= {'/rosout','/parameter_events',
            '/uav/people/follow/status','/uav/people/follow/preview_goal',
            '/uav/people/motion_path','/uav/people/motion_trajectories'}

        # Automatic mode does not need a manually selected RGB box or a START command.
        backend.automatic.policy.config=EpisodeConfig(motion_confirm_s=.05,motion_displacement_m=.001,
            min_samples=3,stationary_confirm_s=.08,observe_s=.08,follow_s=3.,cooldown_s=1.)
        assert ui.set_auto(True)[0]
        pump(.12)
        moving=True
        motion_origin=capture
        pump(.65)
        assert backend.automatic.policy.phase=='SHADOW_FOLLOW',backend.automatic.policy.status()
        moving=False
        until=time.monotonic()+2.
        while backend.automatic.policy.active is not None and time.monotonic()<until:
            pump(.05)
        assert backend.automatic.policy.episodes[-1]['end_reason']=='PERSON_STATIC_OBSERVED',backend.automatic.policy.status()
        assert any(any(len(marker.points)>2 for marker in msg.markers) for msg in trajectory_messages)
        backend.automatic.save(force=True)
        recorded=json.loads((tmp_path/'person-motion.json').read_text())
        assert recorded['episodes'] and recorded['flight_execution'] is False
        assert all(not item['mission_interrupted'] for item in recorded['episodes'])
        completed=[json.loads(line) for line in (tmp_path/'person-motion.jsonl').read_text().splitlines()]
        assert completed[-1]['end_reason']=='PERSON_STATIC_OBSERVED'
        assert all(point['position'][2]==2. for point in completed[-1]['points'])
        assert ui.set_auto(False)[0]
        pump(.1)
        assert not backend.automatic.policy.enabled

        # A stale root command cannot pause a replacement selection.
        old_session=backend.session
        selection.choose([(70,70,130,130)],0,('camera',10.),time.monotonic())
        pump(.25)
        until=time.monotonic()+.6
        while ui.current(selection.sequence) is None and time.monotonic()<until:
            pump(.05)
        request=SetMissionIntent.Request(command='START',coordinator_instance=backend.instance)
        request.mission_uuid.uuid=list(uuid.UUID(old_session).bytes)
        request.request_id.uuid=list(uuid.uuid4().bytes)
        response=backend.intent(request,SetMissionIntent.Response())
        assert not response.accepted and response.reason=='FOLLOW_SESSION_MISMATCH'
        # Window close/crash removes the preview on heartbeat loss, even with healthy sensors.
        accepted,reason=ui.command('START',selection.sequence)
        assert accepted,(reason,backend.core.state,ui.status)
        pump(.1)
        pump(.85,heartbeat=False)
        assert backend.core.state=='LOST' and backend.core.reason=='UI_SELECTION_HEARTBEAT_LOST'
        selection.choose([(70,70,130,130)],0,('camera',capture),time.monotonic())
        pump(.25)
        assert backend.core.state=='READY',backend.core.reason
        # Latest invalid perception must retire an old valid cached observation immediately.
        observations.valid=False
        until=time.monotonic()+.6
        while backend.core.state!='LOST' and time.monotonic()<until:
            pump(.05)
        assert backend.core.state=='LOST' and backend.core.reason=='PERSON_OBSERVATIONS_UNAVAILABLE'
        observations.valid=True
        selection.choose([(70,70,130,130)],0,('camera',capture),time.monotonic())
        pump(.25)
        assert backend.core.state=='READY'
        # Replayed source stamps and selection heartbeats cannot extend input age.
        frozen=True
        clock.clock.sec=13
        pump(.15)
        assert backend.core.state=='LOST' and backend.core.goal is None
    finally:
        backend.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown()
