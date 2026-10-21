"""Native Position handoffs for paused missions and explicit operator cancellation."""
import time
import uuid
from px4_msgs.msg import VehicleStatus

POSITION=VehicleStatus.NAVIGATION_STATE_POSCTL
OFFBOARD=VehicleStatus.NAVIGATION_STATE_OFFBOARD


class PilotIntervention:
    def begin_canceled_manual(self, root_id):
        """Only a completed, explicitly selected root may leave final hold for sticks."""
        if (not root_id or root_id != getattr(self,'last_terminal_goal_id',None)
                or self.active_goal or self.result != ('CANCELED','STOPPED_AND_HOLDING',True)
                or self.owner != 'HOLD_CONTROLLER' or self.phase != 'COMPLETE'
                or self.land_committed or self.runner_lost or self.pilot_abort_reason):
            return False,'CANCEL_MANUAL_REQUIRES_CLEAN_BOUND_TERMINAL'
        gate=getattr(self,'vio_gate',None)
        if (not self.operator_enabled or not self.operator_arm_enabled
                or not self.operator_input_ready() or not self.healthy()
                or gate is None or not getattr(gate,'joint_sim',False)
                or not getattr(gate,'pose_only',False)
                or self.samples['vehicle_status'].nav_state != OFFBOARD
                or self.samples['vehicle_status'].arming_state != 2):
            return False,'CANCEL_MANUAL_REQUIRES_HEALTHY_VIO_FLOW_AND_INPUT'
        self.cancel_manual_state='WAIT_CENTER'
        # The mission is already terminal. Its deadline and pause timeout must
        # never resume work or schedule a normal 30-second post-cancel LAND.
        if hasattr(self,'final_hold_until'):del self.final_hold_until
        self.step_grant=None
        self.return_checkpoint=self.return_home_reference=None
        self.pilot_control=self.pilot_pending=None
        self.segment=None
        if self.planned is not None:self.planned.retire()
        self.change('CANCEL_WAIT_CENTER','CANCELED_HOLD_CENTER_STICKS_AND_THROTTLE')
        return True,'CANCELED_HOLD_WAITING_FOR_CENTERED_STICKS'

    def canceled_manual_tick(self, now):
        state=getattr(self,'cancel_manual_state',None)
        if not state:return False
        status=self.samples.get('vehicle_status')
        if state=='ACTIVE':
            # Native Position and PX4's native failsafes own the aircraft. Never
            # reacquire Offboard or run an old mission after this handoff.
            if self.fresh('vehicle_status',.75) and (status.nav_state != POSITION or status.arming_state != 2):
                self.cancel_manual_state=None
                self.owner='NONE';self.generation+=1
                self.change('CANCELED_MANUAL_EXITED','NATIVE_POSITION_EXITED')
            return True
        if (self.owner!='HOLD_CONTROLLER' or self.clock_fault_latched or not self.healthy()
                or self.resets()!=self.reset_baseline or not self.operator_input_ready()
                or not self.operator_arm_enabled or status.arming_state != 2
                or not self.region(self.position())
                or any(len(self.get_publishers_info_by_topic('/px4_7/fmu/in/'+topic))!=1
                       for topic in ('vehicle_command','trajectory_setpoint','offboard_control_mode'))
                or len(self.get_publishers_info_by_topic('/clock'))!=1):
            self.cancel_manual_state=None
            self.owner='NONE';self.generation+=1
            self.change('FAULT','CANCEL_MANUAL_HEALTH_OR_INPUT_LOST')
            return True  # Withdraw Offboard; preserve native loss protection.
        if status.nav_state==POSITION:
            self.cancel_manual_state='ACTIVE'
            self.owner='PILOT';self.generation+=1
            self.change('CANCELED_MANUAL','CANCELED_NATIVE_VIO_FLOW_POSITION_CONFIRMED')
            return True
        if status.nav_state!=OFFBOARD:
            self.cancel_manual_state=None
            self.owner='NONE';self.generation+=1
            self.change('FAULT','CANCEL_MANUAL_CONTROL_LOST')
            return True
        if state=='POSITION_REQUEST' and now>=self.cancel_manual_transition_until:
            # A rejected mode switch must not leave an unbounded false handoff.
            self.cancel_manual_state=None
            self.final_hold_until=now
            self.change('COMPLETE','CANCEL_MANUAL_POSITION_NOT_CONFIRMED')
            return True  # Existing bounded native LAND fallback on next tick.
        if state=='WAIT_CENTER' and self.pilot_sticks_centered() and self.speed()<=.1:
            self.command(176,1,3)
            self.cancel_manual_state='POSITION_REQUEST'
            self.cancel_manual_transition_until=now+3.
            self.change('CANCEL_POSITION_REQUEST','CANCELED_POSITION_REQUESTED')
        # Maintain the stopped reference until PX4 actually confirms Position.
        self.stream()
        return True

    def pilot_sticks_centered(self):
        return self.operator_input_ready() and all(abs(getattr(
            self.samples['manual_control_setpoint'],key))<=.15
            for key in ('roll','pitch','yaw','throttle'))

    def pilot_request_ready(self):
        if (not self.operator_enabled or not self.operator_arm_enabled or not self.operator_navigation_enabled
                or not self.active_goal or self.result or self.land_committed or self.canceling
                or self.navigation_backend!='EGO' or self.vio_gate is None
                or not getattr(self.vio_gate,'pose_only',False) or not self.healthy()):
            return False,'INTERVENTION_REQUIRES_HEALTHY_VIO_TASK'
        if not self.pilot_sticks_centered():return False,'INTERVENTION_STICKS_AND_THROTTLE_MUST_BE_CENTERED'
        if self.phase not in ('PLAN_REQUEST','NAVIGATE','RETURN','HOVER','PAUSED'):
            return False,'INTERVENTION_PHASE_UNAVAILABLE'
        return True,'INTERVENTION_READY'

    def enter_pilot_control(self):
        ready,reason=self.pilot_request_ready()
        if not ready:return False,reason
        if self.phase!='PAUSED' or self.owner!='HOLD_CONTROLLER' or self.speed()>.1:
            return False,'INTERVENTION_REQUIRES_STOPPED_PAUSE'
        if getattr(self,'pilot_control',None):return False,'INTERVENTION_ALREADY_ACTIVE'
        # A displaced HOVER must rejoin its original anchor through EGO.
        if self.saved_hover is not None:
            self.saved_hover_join=(self.saved_step,tuple(self.target),self.saved_hover)
        self.pilot_control='ENTERING'
        self.pilot_transition_until=time.monotonic()+3.
        self.pilot_pending=None
        self.command(176,1,3)
        self.change('PAUSED','PILOT_POSITION_REQUESTED')
        return True,'PILOT_POSITION_REQUESTED'

    def request_pilot_reacquire(self,verb,now):
        if getattr(self,'pilot_control',None) not in ('ENTERING','ACTIVE'):
            return False,'INTERVENTION_TRANSITION_PENDING'
        if verb not in ('CANCEL','LEASE_LOST'):
            if not self.pilot_sticks_centered():return False,'RESUME_STICKS_AND_THROTTLE_MUST_BE_CENTERED'
            if self.speed()>.1:return False,'INTERVENTION_RESUME_REQUIRES_STOPPED_AIRCRAFT'
            if not self.healthy() or now>=self.pause_until or now>=self.deadline-8.:
                return False,'INTERVENTION_RESUME_HEALTH_OR_DEADLINE'
            try:
                self.planning_ready()
                reference=self.capture_return_home() if verb=='finish' else None
                target=(reference.position_enu[0],reference.position_enu[1],self.position()[2]) if reference else self.target
                grid=self.planned.gate.grid
                radius=self.config['body_radius_m']+self.config['tracking_margin_m']+grid.resolution/2
                if any(not self.region(p) or grid.collision(self.alignment.to_map(p),radius)
                        for p in (self.position(),target)):
                    raise ValueError('PILOT_REJOIN_VOLUME_UNOBSERVED_OR_OCCUPIED')
            except ValueError as error:return False,'INTERVENTION_CONTEXT:'+str(error)
        else:reference=None
        self.pilot_pending=verb
        self.pilot_return_capture=reference
        self.pilot_control='PRESTREAM'
        self.pilot_transition_started=now
        self.reference=self.position()
        self.yaw=float(self.samples['vehicle_local_position'].heading)
        self.owner='PREFLIGHT';self.generation+=1
        self.change('PAUSED','PILOT_OFFBOARD_PRESTREAM')
        return True,'PILOT_REACQUIRE_REQUESTED'

    def pilot_tick(self,now):
        state=getattr(self,'pilot_control',None)
        if not state:return False
        status=self.samples['vehicle_status']
        if state=='BRAKING':
            if self.phase!='PAUSED':return False  # Existing stop/hold confirmation.
            verb=self.pilot_pending
            captured=self.pilot_return_capture
            self.pilot_control=self.pilot_pending=None
            if captured is not None:
                try:
                    if self.px4_home().position_enu!=captured.position_enu:
                        raise ValueError('PX4_RETURN_HOME_CHANGED')
                except ValueError as error:self.fault('PILOT_RETURN:'+str(error));return True
            # Use the same identity-checked service implementation after actual
            # Offboard/stop confirmation; a new internal request cannot replay
            # the earlier asynchronous admission receipt.
            from uav_nav_interfaces.srv import ResumeMission,RequestReturn
            kind=RequestReturn if verb=='finish' else ResumeMission
            request=kind.Request(mission_uuid=self.active_goal.goal_id,coordinator_instance=self.instance)
            request.request_id.uuid=list(uuid.uuid4().bytes)
            response=self.service(verb,request,kind.Response())
            if not response.accepted:self.fault('PILOT_REACQUIRE:'+response.reason)
            return True
        if state in ('ENTERING','ACTIVE'):
            if now>=self.pause_until:
                self.pilot_abort_reason='PILOT_INTERVENTION_TIMEOUT'
                self.canceling=True
                self.request_pilot_reacquire('CANCEL',now)
                return True
            if state=='ENTERING':
                if status.nav_state==POSITION:
                    self.owner='PILOT';self.generation+=1;self.pilot_control='ACTIVE'
                    self.change('PAUSED','PILOT_POSITION_CONFIRMED')
                elif status.nav_state!=OFFBOARD or now>=self.pilot_transition_until:
                    self.fault('PILOT_POSITION_NOT_CONFIRMED')
                else:self.stream()
            elif status.nav_state!=POSITION:self.fault('PILOT_POSITION_CONTROL_LOST')
            return True  # Native Position owns stick control; no Offboard stream.
        if state in ('PRESTREAM','OFFBOARD'):
            if status.nav_state not in (POSITION,OFFBOARD):self.fault('PILOT_REACQUIRE_MODE_CHANGED');return True
            # Track actual current pose until PX4 confirms Offboard; do not fly
            # to the old pause point or run the old trajectory during handoff.
            self.reference=self.position()
            # ROS 2 Offboard proof-of-life is separate from trajectory data.
            # Publishing trajectories in native Position competes with its
            # FlightTask's own uORB setpoints. Only heartbeat until actual
            # Offboard confirmation, then emit the current-pose reference.
            self.offboard_heartbeat()
            if state=='PRESTREAM' and now-self.pilot_transition_started>=1.5:
                self.command(176,1,6);self.pilot_control='OFFBOARD'
                self.pilot_transition_until=now+3.
            elif state=='OFFBOARD':
                if status.nav_state==OFFBOARD:
                    self.stream()
                    self.pilot_control='BRAKING';self.owner='TASK';self.generation+=1
                    self.begin_stop('LEASE_BRAKE' if self.pilot_pending=='LEASE_LOST' else
                        'CANCEL_BRAKE' if self.pilot_pending=='CANCEL' else 'PAUSING')
                elif now>=self.pilot_transition_until:self.fault('PILOT_OFFBOARD_NOT_CONFIRMED')
            return True
        self.fault('INVALID_PILOT_INTERVENTION_STATE');return True
