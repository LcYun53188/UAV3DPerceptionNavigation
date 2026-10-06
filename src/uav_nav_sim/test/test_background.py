from concurrent.futures import Future
from unittest.mock import Mock

from uav_nav_sim.background import BackgroundSelector


def test_only_one_job_can_be_pending_and_completed_slot_is_reusable():
    worker=BackgroundSelector();worker.pool=Mock();future=Future()
    worker.pool.submit.return_value=future
    assert worker.submit(('parent',1),'args')
    assert not worker.submit(('parent',2),'new')
    assert worker.poll() is None
    future.set_result(('goal',False))
    assert worker.poll()==(('parent',1),('goal',False))
    assert worker.future is None
    worker.close()


def test_invalidation_retires_running_result_without_queuing_another_job():
    worker=BackgroundSelector();worker.pool=Mock();future=Future();future.set_running_or_notify_cancel()
    worker.pool.submit.return_value=future
    worker.submit(('parent',1),'args');worker.invalidate()
    assert not worker.submit(('parent',2),'args')
    future.set_result(('oldgoal',False))
    assert worker.poll()==(None,('oldgoal',False))
    assert worker.future is None
    worker.close()


def test_failed_worker_releases_slot_and_retires_pool():
    import pytest
    worker=BackgroundSelector();pool=Mock();worker.pool=pool
    future=Future();pool.submit.return_value=future
    worker.submit('old','args');future.set_exception(RuntimeError('worker failed'))
    with pytest.raises(RuntimeError):worker.poll()
    assert worker.pool is None and worker.future is None and worker.context is None
    pool.shutdown.assert_called_once_with(wait=False,cancel_futures=True)
