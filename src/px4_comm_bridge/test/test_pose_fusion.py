import copy
import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_comm_bridge.pose_fusion import PoseAlignment,convert_aligned_pose,euler_jacobian,FRAME
from px4_comm_bridge.cuvslam_pose import rotation

ID='a'*64
IDENTITY=dict(source_session='vio-source',calibration_id=ID)


def pose(t=12.04,xyz=(2.,3.,.1),angles=(0.,0.,.6),frame='odom'):
    m=PoseWithCovarianceStamped();m.header.frame_id=frame
    m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
    m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=xyz
    r,p,y=[v/2 for v in angles];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    q=m.pose.pose.orientation
    q.w,q.x,q.y,q.z=cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy
    c=np.diag([.01,.02,.03,.004,.005,.006]);c[0,3]=c[3,0]=.001
    m.pose.covariance=c.ravel().tolist()
    return m


def alignment():
    a=PoseAlignment([10.,-4.,.3],-.5,ID)
    a.bind([pose(10+i*.04) for i in range(51)],12.,**IDENTITY,disarmed=True,landed=True)
    return a


def test_alignment_is_fixed_and_preserves_time_cross_covariance_and_input():
    a=alignment();m=pose(xyz=(2.1,3.,.1),angles=(0,0,.7));original=copy.deepcopy(m)
    out=a.apply(m,12.1,**IDENTITY)
    assert m==original and out.header.stamp==m.header.stamp and out.header.frame_id==FRAME
    p=out.pose.pose.position
    assert [p.x,p.y,p.z]==pytest.approx([10+.1*math.cos(-1.1),-4+.1*math.sin(-1.1),.3])
    assert math.atan2(rotation(out.pose.pose.orientation)[1,0],rotation(out.pose.pose.orientation)[0,0])==pytest.approx(-.4)
    c=np.array(out.pose.covariance).reshape(6,6)
    assert np.linalg.eigvalsh(c-np.array(m.pose.covariance).reshape(6,6)).min()>-1e-8
    assert np.trace(c)>np.trace(np.array(m.pose.covariance).reshape(6,6))
    assert abs(c[0,4])>1e-5
    with pytest.raises(ValueError,match='ALREADY_BOUND'):
        a.bind([pose()],12.,**IDENTITY,disarmed=True,landed=True)


@pytest.mark.parametrize('fault',['flying','not_landed','moving','turning','tilted','gap','short','stale','identity','unknown_anchor'])
def test_unsafe_initial_alignment_rejected(fault):
    a=PoseAlignment([0,0,0],0,ID);samples=[pose(10+i*.04) for i in range(51)]
    kw=dict(IDENTITY,disarmed=True,landed=True);now=12.
    if fault=='flying':kw['disarmed']=False
    if fault=='not_landed':kw['landed']=False
    if fault=='moving':samples[-1].pose.pose.position.x+=.1
    if fault=='turning':samples[-1]=pose(12.,angles=(0,0,.7))
    if fault=='tilted':samples=[pose(10+i*.04,angles=(.3,0,0)) for i in range(51)]
    if fault=='gap':samples=samples[:10]+samples[15:]
    if fault=='short':samples=samples[:-1]
    if fault=='stale':now=12.3
    if fault=='identity':kw['calibration_id']='unknown'
    if fault=='unknown_anchor':
        with pytest.raises(ValueError):PoseAlignment([0,0,0],0,'unknown')
        return
    with pytest.raises(ValueError):a.bind(samples,now,**kw)


@pytest.mark.parametrize('fault',['session','reset','calibration','time','stale','frame','covariance'])
def test_bound_alignment_faults_retire_permanently(fault):
    a=alignment();m=pose();kw=dict(IDENTITY)
    if fault=='session':kw['source_session']='new'
    if fault=='reset':kw['reset_counter']=1
    if fault=='calibration':kw['calibration_id']='b'*64
    if fault=='time':m=pose(12.)
    if fault=='stale':m=pose(11.5)
    if fault=='frame':m.header.frame_id='map'
    if fault=='covariance':m.pose.covariance[0]=0.
    with pytest.raises(ValueError):a.apply(m,12.1,**kw)
    with pytest.raises(ValueError):a.apply(pose(12.08),12.1,**IDENTITY)
    assert a.fault


def test_pose_only_output_marks_all_velocity_unknown():
    a=alignment();out=convert_aligned_pose(a.apply(pose(),12.1,**IDENTITY),12.1,reset_counter=3)
    assert out.velocity_frame==out.VELOCITY_FRAME_UNKNOWN
    assert all(math.isnan(v) for name in ('velocity','velocity_variance','angular_velocity') for v in getattr(out,name))
    assert out.timestamp_sample==12040000 and out.reset_counter==3
    assert list(out.position)==pytest.approx([-4,10,-.3])
    assert sum(v*v for v in out.q)==pytest.approx(1.,abs=1e-6)


def test_euler_variance_jacobian_matches_independent_finite_rotations():
    r=rotation(pose(angles=(.2,-.3,.8)).pose.pose.orientation)
    def euler(r):return np.array([math.atan2(r[2,1],r[2,2]),math.asin(-r[2,0]),math.atan2(r[1,0],r[0,0])])
    numerical=np.zeros((3,3));h=1e-6
    for i in range(3):
        axis=np.eye(3)[i];x,y,z=axis;skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        perturb=np.eye(3)+math.sin(h)*skew+(1-math.cos(h))*(skew@skew)
        numerical[:,i]=(euler(perturb@r)-euler(perturb.T@r))/(2*h)
    assert euler_jacobian(r)==pytest.approx(numerical,abs=1e-8)


def test_euler_yaw_variance_uses_full_fixed_axis_covariance():
    m=pose(angles=(.2,-.3,.8),frame=FRAME)
    c=np.array(m.pose.covariance).reshape(6,6);c[3,5]=c[5,3]=.001;m.pose.covariance=c.ravel().tolist()
    out=convert_aligned_pose(m,12.1)
    n=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    j=euler_jacobian(n@rotation(m.pose.pose.orientation)@np.diag([1,-1,-1]))
    assert list(out.orientation_variance)==pytest.approx(np.diag(j@n@c[3:,3:]@n.T@j.T),rel=1e-6)
    assert out.orientation_variance[2]!=pytest.approx(c[5,5])


def test_alignment_bound_covers_unknown_correlated_initial_and_current_errors():
    rng=np.random.default_rng(42)
    for _ in range(10):
        l=rng.normal(size=(12,12))*.003;joint=l@l.T
        a=PoseAlignment([0,0,0],-.5,ID)
        samples=[pose(10+i*.04) for i in range(51)]
        for m in samples:m.pose.covariance=joint[:6,:6].ravel().tolist()
        a.bind(samples,12.,**IDENTITY,disarmed=True,landed=True)
        current=pose(xyz=(2.1,3.1,.2));current.pose.covariance=joint[6:,6:].ravel().tolist()
        result=a.apply(current,12.1,**IDENTITY)
        jc=np.zeros((6,6));jc[:3,:3]=jc[3:,3:]=a.r
        displacement=a.r@np.array([.1,.1,.1]);b=np.cross([0,0,1],displacement)
        ji=np.zeros((6,6));ji[:3,:3]=-a.r
        ji[:3,3:]=-np.outer(b,a.initial_yaw_jacobian)
        ji[3:,3:]=-np.outer([0,0,1],a.initial_yaw_jacobian)
        exact=np.column_stack([ji,jc])@joint@np.column_stack([ji,jc]).T
        bound=np.array(result.pose.covariance).reshape(6,6)
        assert np.linalg.eigvalsh(bound-exact).min()>-1e-12


def test_bound_initial_covariance_is_frozen_when_caller_reuses_message():
    a=PoseAlignment([0,0,0],0,ID);samples=[pose(10+i*.04) for i in range(51)]
    a.bind(samples,12.,**IDENTITY,disarmed=True,landed=True)
    saved=a.initial_covariance.copy()
    samples[-1].pose.covariance[0]=.2
    assert a.initial_covariance==pytest.approx(saved)
