"""Actual QGC/uinput -> PX4 acceptance stimuli, never fabricated FMU telemetry."""
import math
import time
from px4_msgs.msg import VehicleStatus
from sim_validation import write_json


CASES=('full','finish-mission','pilot-intervention','local-land-manual','local-land-position')
INPUT_CASES=CASES[2:]


class NoGnssCase:
    def __init__(self,name,node,directory,truth):
        self.name,self.node,self.directory,self.truth=name,node,directory,truth
        self.evidence={};self.channels=[0.,0.,0.,0.,1.,1.,0.,-1.,-1.,-1.,-1.]+[0.]*5

    def input(self,**channels):
        for channel,value in channels.items():self.channels[int(channel.removeprefix('ch'))-1]=value
        write_json(self.directory/'validation-input.pending.json',dict(channels=self.channels))
        (self.directory/'validation-input.pending.json').replace(self.directory/'validation-input.json')

    def until(self,predicate,seconds,reason):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if predicate():return
            time.sleep(.02)
        raise RuntimeError(reason)

    def operator(self):
        self.input()
        self.until(self.node.pilot_sticks_centered,5,'Actual calibrated QGC neutral input unavailable')
        self.node.operator_enabled=True
        self.node.operator_arm_enabled=self.node.operator_navigation_enabled=True
        flags=self.node.vio_gate.samples['flags']
        if flags.cs_gnss_pos or flags.cs_gnss_vel:raise RuntimeError('GNSS aiding active in no-GNSS acceptance')
        self.evidence.update(input_source='actual virtual uinput via calibrated QGC; not physical Pocket',
            gnss_position_fused=bool(flags.cs_gnss_pos),gnss_velocity_fused=bool(flags.cs_gnss_vel),
            actual_manual_source=self.node.samples['manual_control_setpoint'].data_source)

    def missing_home_refusal(self):
        """Negative reader-cache visibility injection; no forged DDS or Home."""
        import uuid
        from uav_nav_interfaces.srv import RequestReturn
        from unique_identifier_msgs.msg import UUID
        node=self.node
        with node.lock:
            original=node.samples.pop('home_position',None)
            if original is None:raise RuntimeError('Actual native Home missing before negative test')
            before=(node.owner,node.generation,node.phase,bytes(node.child.uuid),node.step_index,
                node.deadline,len(node.commands))
            try:
                response=node.request_return(RequestReturn.Request(mission_uuid=node.active_goal.goal_id,
                    coordinator_instance=node.instance,request_id=UUID(uuid=list(uuid.uuid4().bytes)),
                    finish_mission=True),RequestReturn.Response())
                after=(node.owner,node.generation,node.phase,bytes(node.child.uuid),node.step_index,
                    node.deadline,len(node.commands))
            finally:node.samples['home_position']=original  # Original bytes/time, never renewed.
        refused=(not response.accepted and response.reason=='RETURN_CONTEXT_INVALID:PX4_LOCAL_HOME_UNAVAILABLE'
            and before==after)
        self.evidence.update(missing_home_refused_without_control_mutation=refused,
            negative_injection='gateway Home cache visibility removed; no synthetic FMU publication',
            original_home_timestamp=int(original.timestamp))
        if not refused:raise RuntimeError('Missing Home request was not rejected atomically: '+response.reason)

    def local_land(self):
        self.operator();node=self.node
        self.input(ch6=-1. if self.name.endswith('manual') else 0.,ch7=1.)
        mode='ACRO' if self.name.endswith('manual') else 'VIO_POSITION'
        with node.lock:accepted,reason=node.operator_command(mode)
        if not accepted:raise RuntimeError('Manual takeover rejected: '+reason)
        expected=VehicleStatus.NAVIGATION_STATE_ACRO if mode=='ACRO' else VehicleStatus.NAVIGATION_STATE_POSCTL
        self.until(lambda:node.active_goal is None and node.samples['vehicle_status'].nav_state==expected,
            5,'Actual pilot mode takeover not confirmed')
        origin=self.truth[-1]['position'];started=time.monotonic()
        self.input(ch8=1.)
        with node.lock:accepted,reason=node.operator_command('LAND')
        if not accepted:raise RuntimeError('Native local LAND rejected: '+reason)
        self.until(lambda:node.samples['vehicle_land_detected'].landed and
            node.samples['vehicle_status'].arming_state==1,35,'Native landing/disarm not confirmed')
        points=[p for p in self.truth if p['mono']>=started]
        drift=max((math.hypot(p['position'][0]-origin[0],p['position'][1]-origin[1]) for p in points),default=999.)
        rates=[(a['position'][2]-b['position'][2])/(b['source_sim_s']-a['source_sim_s']) for a,b in zip(points,points[1:])
            if .4<a['position'][2]<origin[2]-.2 and b['source_sim_s']-a['source_sim_s']>.001]
        from statistics import median
        speed=median(rates) if rates else None
        self.evidence.update(native_pilot_mode=expected,land_reason=reason,
            horizontal_drift_m=drift,descent_median_mps=speed,duration_s=time.monotonic()-started,
            landed_disarmed=True,passed=drift<=.3 and speed is not None and .1<speed<.45)

    def intervene(self,intent):
        self.operator();node=self.node
        root=bytes(node.active_goal.goal_id.uuid);home=node.home;deadline=node.deadline
        self.input(ch7=1.)
        intent('PAUSE')
        self.until(lambda:node.phase=='PAUSED',10,'Task did not stop before pilot handoff')
        with node.lock:accepted,reason=node.enter_pilot_control()
        if not accepted:raise RuntimeError('Pilot handoff rejected: '+reason)
        self.until(lambda:node.pilot_control=='ACTIVE',4,'Native Position not confirmed')
        outputs=node.output_count;origin=self.truth[-1]['position'];start=time.monotonic()
        # Small bounded stick pulse. PX4 consumes the actual QGC channel data;
        # truth is an independent acceptance observation, never a setpoint.
        heading=float(node.samples['vehicle_local_position'].heading)
        # Fly toward the already observed launch side, rather than beyond the
        # target's protected volume. The direction uses actual VIO/EKF yaw.
        pulse=({'ch1':math.copysign(.35,math.sin(heading))} if abs(math.sin(heading))>.7 else
               {'ch2':-math.copysign(.35,math.cos(heading))})
        self.input(**pulse)
        try:
            self.until(lambda:math.dist(self.truth[-1]['position'],origin)>.3,2.5,
                'No actual displacement from virtual stick pulse')
        finally:self.input(ch1=0.,ch2=0.)
        self.until(lambda:node.speed()<.1,5,'Pilot neutral did not stop the aircraft')
        shift=math.dist(self.truth[-1]['position'],origin)
        quiet=node.output_count==outputs
        self.input(ch7=0.)
        intent('RESUME')
        self.until(lambda:node.pilot_control is None and node.phase in ('PLAN_REQUEST','NAVIGATE','HOVER'),10,
            'Task did not replan after pilot intervention')
        self.evidence.update(native_position_confirmed=True,offboard_reconfirmed=node.samples['vehicle_status'].nav_state==14,
            pilot_stick_pulse=pulse,pilot_initial_heading_ned=heading,
            manual_displacement_m=shift,manual_duration_s=time.monotonic()-start,
            no_automatic_stream_during_pilot=quiet,
            original_root_home_deadline_preserved=bytes(node.active_goal.goal_id.uuid)==root and node.home==home and node.deadline==deadline,
            passed=quiet and .3<=shift<=1.2 and node.samples['vehicle_status'].nav_state==14
                and bytes(node.active_goal.goal_id.uuid)==root and node.home==home and node.deadline==deadline)
