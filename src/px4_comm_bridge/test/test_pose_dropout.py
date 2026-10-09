from px4_comm_bridge.pose_dropout import can_drop


def check(**kw):
    args=dict(bound=True,ros=10.10,mono=20.10,last_sample=10.,last_receive=20.)
    args.update(kw)
    return can_drop('bounded_gap','VIO_UNCERTAINTY_INVALID',**args)


def test_one_uncertain_sample_does_not_extend_last_good_deadline():
    assert check()
    assert not check(ros=10.201)
    assert not check(mono=20.201)


def test_identity_tracking_reset_and_other_faults_are_not_dropped():
    args=dict(bound=True,ros=10.1,mono=20.1,last_sample=10.,last_receive=20.)
    for reason in ('VIO_RESET_REQUESTED','VIO_TRACKING_INVALID','VIO_TIME_DISCONTINUITY','VIO_WRITER_COUNT'):
        assert not can_drop('bounded_gap',reason,**args)
    assert not can_drop('strict','VIO_UNCERTAINTY_INVALID',**args)
    assert not check(bound=False)
    assert not check(last_sample=None)
    assert not check(last_receive=None)
