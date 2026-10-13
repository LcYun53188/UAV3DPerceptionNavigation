"""Explicit disarmed x500 physics fixture; this policy never admits flight."""
import time
import xml.etree.ElementTree as ET
import numpy as np
from std_msgs.msg import Bool
from px4_comm_bridge.pose_stream import AlignedPoseStream
from px4_vio_real_fusion import RealPoseFusionAudit, json_message
from vio_mtf01_joint import JointPoseFusionAudit


class DisarmedFixtureStream(AlignedPoseStream):
    fresh_disarmed = False

    def state_allowed(self, ground, armed):
        # Initial binding still requires the actual landed state. Afterwards only
        # this disarmed test policy can follow the wrench fixture off the ground.
        return self.fresh_disarmed and not armed and (ground or self.bound)


class FixtureState:
    stream_class = DisarmedFixtureStream

    def state_keywords(self):
        now=time.monotonic();ros=self.ros()
        fresh=True
        for key,age in (('vehicle',.75),('land',1.5)):
            m=self.last.get(key)
            fresh &= bool(m is not None and now-self.received[key]<=age and
                -.05<=ros-m.timestamp/1e6<=age and
                len(self.node.get_publishers_info_by_topic(self.topics[key]))==1)
        self.stream.fresh_disarmed=bool(fresh and self.last['vehicle'].arming_state==1)
        return dict(ground=self.ground())


class DynamicPoseFusionAudit(FixtureState,RealPoseFusionAudit): pass
class DynamicJointFusionAudit(FixtureState,JointPoseFusionAudit): pass


def attach_fixture(directory,profile,models,plugin,*,attitude_amplitude=0.):
    if attitude_amplitude not in (0.,.12):raise ValueError('Unreviewed fixture attitude amplitude')
    path=directory/profile['model']/'model.sdf'
    tree=ET.parse(path);model=tree.getroot().find('model')
    if model.findtext('include/uri')!='model://x500':raise ValueError('Fixture requires actual x500')
    # Includes all rotor and attached sensor masses, from the actual SDF inputs.
    mass=sum(float(e.text) for e in ET.parse(models/'x500_base/model.sdf').findall('.//inertial/mass'))
    mass+=sum(float(e.text) for e in model.findall('.//inertial/mass'))
    element=ET.SubElement(model,'plugin',filename=str(plugin),name='uav::test::SameAircraftMotion')
    ET.SubElement(element,'compound_mass').text=repr(mass)
    ET.SubElement(element,'attitude_amplitude').text=repr(attitude_amplitude)
    truth=ET.SubElement(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for k,v in [('odom_frame','world'),('robot_base_frame','base_link'),('odom_topic','/vio/truth'),
                ('odom_covariance_topic','/vio/truth_covariance_observer_only'),
                ('tf_topic','/vio/truth_tf_observer_only'),
                ('odom_publish_frequency','100'),('dimensions','3')]:ET.SubElement(truth,k).text=v
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
    return dict(schema=1,scope='test-only actual disarmed x500 wrench; not motor flight',
        compound_mass_kg=mass,trajectory_duration_s=100,minimum_final_rest_s=20,
        roll_pitch_amplitude_rad=attitude_amplitude,
        trigger_topic='/vio/fixture/start',truth_covariance_topic='/vio/truth_covariance_observer_only',
        px4_default_truth_covariance_topic_forbidden=True,flight_authorized=False)


class FixtureTrigger:
    def __init__(self,node,*,attitude_amplitude=0.):
        self.node=node;self.publisher=node.create_publisher(Bool,'/vio/fixture/start',10)
        self.started=None
        self.attitude_amplitude=attitude_amplitude

    def tick(self,fusion,motion,*,truth_isolated=False):
        ros=fusion.ros()
        if not truth_isolated or self.started is not None or ros<45 or fusion.longest_ready_s<5 or not fusion.ground():return
        if fusion.gate.reason!='READY' or fusion.stream.fault or not fusion.controls_valid():return
        if hasattr(fusion,'mtf_health') and not fusion.mtf_health()[0]:return
        if len(motion.truth)<100 or self.publisher.get_subscription_count()!=1:return
        self.publisher.publish(Bool(data=True))
        self.started=dict(ros=ros,mono=time.monotonic(),ground=True,disarmed=True,
            alignment_id=fusion.stream.alignment.alignment_id,gate_reason=fusion.gate.reason,
            source_count=len(fusion.outputs))

    def result(self,motion,fusion):
        checks=dict(triggered=self.started is not None)
        if not self.started:return dict(passed=False,checks=checks,scope='disarmed dynamic fixture')
        start=self.started['ros'];end=start+100
        truth=[r for r in motion.truth if start+12<=r['stamp']<=start+88]
        rest=[r for r in motion.truth if r['stamp']>=end+2]
        if not truth or not rest:return dict(passed=False,checks=dict(checks,motion_and_rest_present=False),trigger=self.started)
        positions=np.asarray([r['position'] for r in truth]);final=np.asarray([r['position'] for r in rest])
        origin=np.asarray(min(motion.truth,key=lambda r:abs(r['stamp']-start))['position'])
        from px4_vio_motion_audit import rotation
        yaws=np.unwrap([np.arctan2(rotation(r['quaternion'])[1,0],rotation(r['quaternion'])[0,0]) for r in truth])
        rotations=np.asarray([rotation(r['quaternion']) for r in truth])
        roll_pitch=np.column_stack((np.arctan2(rotations[:,2,1],rotations[:,2,2]),
                                    -np.arcsin(np.clip(rotations[:,2,0],-1,1))))
        history=fusion.history_messages() if hasattr(fusion,'history_messages') else fusion.history
        dynamic=[r for r in history if hasattr(r['message'],'timestamp') and
                 start+12<=r['message'].timestamp/1e6<=start+88]
        counts={k:len({r['message'].timestamp_sample for r in dynamic if r['name']==k and
                      getattr(r['message'],'fused',False)}) for k in fusion.gate.aids}
        flags=[json_message(r['message']) for r in dynamic if r['name']=='flags']
        rejected=('reject_hor_pos','reject_ver_pos','reject_yaw','reject_optflow_x','reject_optflow_y')
        checks.update(three_axis_excitation=bool(np.all(np.ptp(positions,axis=0)>=[.8,.6,.3])),
            yaw_excitation=bool(np.ptp(yaws)>=.6),final_rest=rest[-1]['stamp']-rest[0]['stamp']>=20,
            return_to_origin=bool(np.linalg.norm(final[-1]-origin)<=.05),
            stationary_final_rest=bool(np.max(np.linalg.norm(final-final[0],axis=1))<=.01),
            actual_dynamic_ev_fusion=all(n>=100 for n in counts.values()),
            dynamic_flags_present=len(flags)>=50,
            ev_continues_through_motion=bool(fusion.outputs and
                fusion.outputs[-1]['ev'].timestamp_sample/1e6>=end+20),
            no_dynamic_innovation_rejection=bool(flags and not any(any(f.get(k,False) for k in rejected) for f in flags)),
            no_dynamic_gnss=bool(flags and all(not any(f.get(k,False) for k in ('cs_gnss_pos','cs_gnss_vel','cs_gnss_hgt','cs_gps')) for f in flags)),
            disarmed_at_end=fusion.terminal_ground() if hasattr(fusion,'terminal_ground') else fusion.ground(),
            no_control_writers=fusion.controls_valid())
        if self.attitude_amplitude:
            final_rotations=np.asarray([rotation(r['quaternion']) for r in rest])
            rotation_error=lambda a,b:float(np.arccos(np.clip((np.trace(a@b.T)-1)/2,-1,1)))
            launch_rotation=rotation(min(motion.truth,key=lambda r:abs(r['stamp']-start))['quaternion'])
            checks.update(roll_pitch_excitation=bool(np.all(np.ptp(roll_pitch,axis=0)>=.16)),
                return_to_launch_attitude=rotation_error(final_rotations[-1],launch_rotation)<=.01,
                stationary_final_attitude=max(rotation_error(r,final_rotations[0]) for r in final_rotations)<=.01)
        return dict(passed=all(checks.values()),checks=checks,trigger=self.started,
            scope='same-aircraft disarmed fixture dynamic fusion, NOT motor flight',flight_authorized=False,
            truth_extent_m=np.ptp(positions,axis=0).tolist(),yaw_extent_rad=float(np.ptp(yaws)),
            roll_pitch_extent_rad=np.ptp(roll_pitch,axis=0).tolist(),
            return_error_m=float(np.linalg.norm(final[-1]-origin)),final_rest_s=rest[-1]['stamp']-rest[0]['stamp'],
            dynamic_fused_counts=counts,innovation_rejection_samples=sum(any(f.get(k,False) for k in rejected) for f in flags))
