"""Paired normalized VIO source -> fixed launch alignment, with terminal retirement.

No ROS graph access or publishers. The caller supplies verified endpoint GIDs and
fresh ground state. This stream is deliberately disarmed-audit-only.
"""
from .pose_fusion import PoseAlignment, checked_pose, convert_aligned_pose
from .vio_input import SourceContinuity, stamp_s


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class AlignedPoseStream:
    def __init__(self,calibration,position,yaw,anchor_id):
        self.calibration=calibration
        self.alignment=PoseAlignment(position,yaw,anchor_id)
        self.continuity=SourceContinuity()
        self.identity=None
        self.window=[]
        self.last_receive=None
        self.last_clock=None
        self.fault=''
        self.reason='VIO_ALIGNMENT_MISSING'
        self.count=0

    @property
    def bound(self):
        return self.alignment.binding is not None

    def reject(self,reason):
        if self.bound: self.fault=self.fault or reason
        self.reason=self.fault or reason
        if not self.bound:
            self.window=[]
            self.continuity=SourceContinuity()
            self.last_receive=None
        return None

    def check(self,ros,mono,*,ground):
        if self.last_clock is not None and ros<self.last_clock:
            self.fault=self.fault or 'VIO_CLOCK_RESET'
        self.last_clock=ros
        if self.fault: return self.reject(self.fault)
        if not ground: return self.reject('VIO_ALIGNMENT_REQUIRES_GROUND')
        if self.bound and (self.last_receive is None or mono-self.last_receive>.2):
            return self.reject('VIO_RECEIVE_GAP')
        if self.bound and not -.05<=ros-self.alignment.last_stamp<=.2:
            return self.reject('VIO_SAMPLE_STALE')
        return True

    def accept(self,pose,status,ros,mono,*,pose_gid,status_gid,ground):
        if not self.check(ros,mono,ground=ground): return None
        try:
            if (not status.valid or status.reason or status.header.frame_id!='odom'
                    or status.calibration_id!=self.calibration or not status.localization_session
                    or not -.05<=ros-stamp_s(status.header.stamp)<=.2):
                raise ValueError('VIO_SOURCE_INVALID')
            if stamp_ns(pose.header.stamp)!=stamp_ns(status.sample_stamp):
                raise ValueError('VIO_SAMPLE_PAIR_INVALID')
            if not pose_gid or not status_gid: raise ValueError('VIO_WRITER_INVALID')
            identity=(status.localization_session,status.calibration_id,status.reset_counter,pose_gid,status_gid)
            if self.identity is not None and identity!=self.identity:
                self.fault=self.fault or 'VIO_SOURCE_IDENTITY_CHANGED'
                raise ValueError(self.fault)
            checked_pose(pose,ros,'odom')
            self.continuity.accept(pose,pose_gid)
            self.identity=identity
            self.last_receive=mono
            kw=dict(source_session=status.localization_session,calibration_id=status.calibration_id,
                    reset_counter=status.reset_counter)
            if not self.bound:
                self.window.append(pose)
                while len(self.window)>1 and stamp_s(pose.header.stamp)-stamp_s(self.window[1].header.stamp)>=2.:
                    self.window.pop(0)
                self.reason='VIO_ALIGNMENT_STABILIZING'
                if len(self.window)>=40 and stamp_s(pose.header.stamp)-stamp_s(self.window[0].header.stamp)>=2.:
                    self.alignment.bind(self.window,ros,**kw,disarmed=ground,landed=ground)
                return None  # Never emit the anchor sample twice.
            aligned=self.alignment.apply(pose,ros,**kw)
            converted=convert_aligned_pose(aligned,ros,status.reset_counter)
            self.reason=''
            self.count+=1
            return aligned,converted
        except ValueError as exc:
            return self.reject(str(exc))
