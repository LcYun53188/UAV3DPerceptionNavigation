from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_pose_window import source_window


def test_same_source_history_has_same_rate_evidence_at_both_pacings():
    samples=[dict(stamp=10.+i*.04,mono=100.+i*.04) for i in range(150)]
    paced=[dict(stamp=p['stamp'],mono=100.+(p['mono']-100.)/.8) for p in samples]
    normal=source_window(samples,16.,107.)
    slow=source_window(paced,16.,109.)
    assert [p['stamp'] for p in normal]==[p['stamp'] for p in slow]
    assert len(normal)==125


def test_stalled_ros_cannot_use_poses_received_after_reset_boundary():
    poses=[dict(stamp=10.,mono=99.),dict(stamp=10.,mono=101.)]
    assert source_window(poses,10.,100.)==poses[:1]


def test_old_or_future_samples_are_not_rate_evidence():
    poses=[dict(stamp=4.99,mono=99.),dict(stamp=10.01,mono=99.)]
    assert not source_window(poses,10.,100.)
