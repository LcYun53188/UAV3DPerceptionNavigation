"""Ground-only EV horizontal fusion transaction; no flight or sensor publishers.

Both profiles retain VIO height/yaw and continuous optical-flow/range inputs.
Pinned PX4 resets XY when EV position starts without GNSS: air changes are denied.
"""
import math
from .vio_gate import VioGate


PROFILES = {'VIO_FLOW': 11, 'FLOW_HORIZONTAL': 10}


class LocalizationSelector:
    def __init__(self, calibration_id, backend, *, timeout_s=8.):
        if not math.isfinite(timeout_s) or not 3 <= timeout_s <= 20:
            raise ValueError('INVALID_SWITCH_TIMEOUT')
        self.calibration_id, self.backend, self.timeout_s = calibration_id, backend, timeout_s
        self.samples, self.received, self.decisions, self.events = {}, {}, {}, []
        self.active_profile = self.requested_profile = 'VIO_FLOW'
        self.phase, self.reason, self.fault = 'IDLE', 'SOURCE_WARMUP', ''
        self.generation, self.request_id = 0, bytes(16)
        self.ready = False
        self.future = None
        self.source_identity = self.selector_identity = None
        self.endpoint_identity = None
        self.last_ros = self.clock_advance = None
        self.reset_identity = None
        self._new_gates()

    def _new_gates(self):
        self.gates = {name: VioGate(self.calibration_id, fusion_profile=profile) for name,profile in
            [('VIO_FLOW','aligned_pose_mtf_sim_v1'),('FLOW_HORIZONTAL','flow_horizontal_mtf_sim_v1')]}
        for name,message in self.samples.items():
            for gate in self.gates.values():gate.receive(name,message,self.received[name])

    def receive(self, name, message, now):
        if self.source_identity is not None and name=='source':
            identity=(message.localization_session,message.reset_counter,message.calibration_id)
            if identity!=self.source_identity or not message.valid:
                self.fault='SOURCE_IDENTITY_OR_TRACKING_LOST'
        if self.selector_identity is not None and name=='selector':
            if (message.primary_instance,message.instance_changed_count)!=self.selector_identity:
                self.fault='EKF_INSTANCE_CHANGED'
        self.samples[name],self.received[name]=message,now
        if name=='local':
            current=tuple(getattr(message,n) for n in
                ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter'))
            if self.reset_identity is not None and current!=self.reset_identity:
                if self.phase in ('APPLYING','VERIFYING','ROLLING_BACK','VERIFYING_ROLLBACK'):
                    # Only an explicit, continuously proven ground transaction can
                    # establish a new frame. No active Action may be present.
                    self.reset_identity=current
                    self._new_gates()
                elif self.source_identity is None:self._new_gates()  # Unbound EKF startup.
                else:self.fault='LOCALIZATION_RESET_OUTSIDE_SWITCH'
            self.reset_identity=current
        for gate in self.gates.values():gate.receive(name,message,now)

    def fail(self, reason):
        self.fault=reason
        self.phase,self.reason,self.ready='FAULT',reason,False

    def request(self, profile, request_id, now, ground_safe):
        key=bytes(request_id)
        if not any(key):return False,'INVALID_REQUEST_ID'
        if key in self.decisions:
            old,decision=self.decisions[key]
            return decision if old==profile else (False,'REQUEST_ID_CONFLICT')
        if len(self.decisions)>=128:return False,'REQUEST_LIMIT'
        decision=(False,'INVALID_PROFILE')
        if profile in PROFILES:
            if self.fault:decision=(False,self.fault)
            elif not ground_safe:decision=(False,'GROUND_DISARMED_IDLE_REQUIRED')
            elif self.phase not in ('IDLE','ROLLED_BACK'):decision=(False,'SWITCH_BUSY')
            elif not self.ready:decision=(False,'CURRENT_FUSION_NOT_READY')
            elif profile==self.active_profile:decision=(True,'ALREADY_ACTIVE')
            else:
                self.request_id=key
                self.requested_profile=profile
                self.deadline=now+self.timeout_s
                self.phase,self.reason,self.ready='APPLYING','PARAMETER_REQUESTED',False
                self.future=self.backend.submit(PROFILES[profile])
                self.events.append(dict(phase=self.phase,target=profile,mono=now,request_id=key.hex()))
                decision=(True,'SWITCH_ADMITTED')
        self.decisions[key]=(profile,decision)
        return decision

    def _rollback(self, now, reason):
        self.rollback_reason=reason
        self.deadline=now+self.timeout_s
        self.requested_profile=self.active_profile
        self.phase,self.reason,self.ready='ROLLING_BACK',reason,False
        self.future=self.backend.submit(PROFILES[self.active_profile])
        self._new_gates()
        self.events.append(dict(phase=self.phase,reason=reason,mono=now))

    def tick(self, now, ros, writers, ground_safe, endpoints=None, *, native_landing=False):
        if self.last_ros is not None and ros<self.last_ros:self.fault='CLOCK_RESET'
        if ros>0 and (self.last_ros is None or ros>self.last_ros):self.clock_advance=now
        self.last_ros=ros
        if self.clock_advance is not None and now-self.clock_advance>.5:self.fault='CLOCK_STALLED'
        if self.endpoint_identity is not None and endpoints is not None and endpoints!=self.endpoint_identity:
            self.fault='TELEMETRY_ENDPOINT_CHANGED'
        if self.fault:self.fail(self.fault);return False
        busy=self.phase in ('APPLYING','VERIFYING','ROLLING_BACK','VERIFYING_ROLLBACK')
        if busy and not ground_safe:
            self.fail('GROUND_AUTHORITY_LOST_DURING_SWITCH');return False
        if self.phase in ('APPLYING','ROLLING_BACK') and self.future.done():
            try:
                receipt=self.future.result()
                if receipt['after']['EKF2_EV_CTRL']!=PROFILES[self.requested_profile]:
                    raise ValueError('PARAMETER_READBACK_MISMATCH')
                self.events.append(dict(phase='PARAMETER_READBACK',mono=now,receipt=receipt))
                self.phase='VERIFYING' if self.phase=='APPLYING' else 'VERIFYING_ROLLBACK'
                self.reason='WAITING_FOR_ACTUAL_FUSION'
                self._new_gates()
            except Exception as error:
                if self.phase=='ROLLING_BACK':self.fail('ROLLBACK_PARAMETER_FAILED:'+str(error));return False
                self._rollback(now,'PARAMETER_FAILED:'+str(error))
        if busy and now>=self.deadline:
            if self.phase in ('ROLLING_BACK','VERIFYING_ROLLBACK'):
                self.fail('ROLLBACK_FUSION_TIMEOUT');return False
            self._rollback(now,'TARGET_FUSION_TIMEOUT')
        profile=self.requested_profile if self.phase in ('VERIFYING','VERIFYING_ROLLBACK') else self.active_profile
        gate=self.gates[profile]
        fused=gate.ready(now,ros,writers,native_landing=native_landing and not busy)
        if self.phase in ('VERIFYING','VERIFYING_ROLLBACK') and fused:
            rolled_back=self.phase=='VERIFYING_ROLLBACK'
            if not rolled_back:
                self.active_profile=profile
                self.generation+=1
            self.phase='ROLLED_BACK' if rolled_back else 'IDLE'
            self.reason=self.rollback_reason if rolled_back else 'FUSION_CONFIRMED'
            self.events.append(dict(phase=self.phase,mono=now,profile=profile,generation=self.generation))
        self.ready=fused and self.phase in ('IDLE','ROLLED_BACK')
        if self.ready:
            source=self.samples['source'];selector=self.samples['selector']
            self.source_identity=(source.localization_session,source.reset_counter,source.calibration_id)
            self.selector_identity=(selector.primary_instance,selector.instance_changed_count)
            if endpoints is not None and self.endpoint_identity is None:self.endpoint_identity=endpoints
            if self.reason=='SOURCE_WARMUP':self.reason='FUSION_CONFIRMED'
        elif self.phase in ('IDLE','ROLLED_BACK'):self.reason=gate.reason
        return self.ready

    @property
    def navigation_allowed(self):
        return self.ready and self.active_profile=='VIO_FLOW'

    @property
    def health_gate(self):
        return self.gates[self.active_profile]
