from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
from uav_nav_interfaces.msg import MapSnapshot
from uav_nav_sim.executor import Executor
from uav_nav_sim.map_session import MapSession


def test_old_epoch_cannot_restore_a_session():
    state=NS(session=('current',20),map=None,grid=None,curve=None,stop=Mock())
    message=MapSnapshot(map_id='old',epoch=19,valid=False)
    Executor.map_cb(state,message)
    assert state.session==('current',20)
    state.stop.assert_not_called()


def test_map_id_change_requires_new_epoch():
    state=NS(session=('current',20),map=None,grid=None,curve=None,stop=Mock())
    Executor.map_cb(state,MapSnapshot(map_id='other',epoch=20,valid=False))
    assert state.session==('current',20)


def test_new_epoch_cancels_goal_even_when_stopped():
    state=NS(session=('current',20),map=None,grid=None,curve=None,stop=Mock(),last_id=9)
    Executor.map_cb(state,MapSnapshot(map_id='loaded',epoch=21,valid=False))
    assert state.session==('loaded',21)
    assert state.last_id==0
    state.stop.assert_called_once_with('MAP_SESSION_CHANGED')


def test_loading_resets_version_before_invalid_notification():
    state=NS(busy=False,executor_state='HOLD',executor_received=time.monotonic(),epoch=20,version=100)
    seen=[]
    state.invalidate=lambda:seen.append((state.epoch,state.version,state.busy))
    MapSession.begin_operation(state)
    assert seen==[(21,0,True)]


def test_map_operation_rejects_execution():
    import pytest
    state=NS(busy=False,executor_state='EXECUTING',executor_received=time.monotonic())
    with pytest.raises(ValueError):MapSession.begin_operation(state)


def test_loaded_map_identity_is_published_with_new_epoch():
    state=NS(busy=False,executor_state='HOLD',executor_received=time.monotonic(),
             map_id='startup',epoch=20,version=100)
    consumer=NS(session=('startup',20),map=None,grid=None,curve=None,stop=Mock(),last_id=9)
    state.invalidate=lambda:Executor.map_cb(consumer,MapSnapshot(
        map_id=state.map_id,epoch=state.epoch,version=state.version,valid=False))
    MapSession.begin_operation(state,map_id='saved')
    assert consumer.session==('saved',21)
    assert consumer.last_id==0
    consumer.stop.assert_called_once_with('MAP_SESSION_CHANGED')


def test_rejected_load_does_not_change_map_identity():
    import pytest
    state=NS(busy=False,executor_state='EXECUTING',executor_received=time.monotonic(),
             map_id='current',epoch=20,version=100)
    with pytest.raises(ValueError):MapSession.begin_operation(state,map_id='saved')
    assert (state.map_id,state.epoch,state.version)==('current',20,100)
