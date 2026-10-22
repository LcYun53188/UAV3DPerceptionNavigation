import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts/compat'))
from usb_joystick import SwitchEdges
from pocket_switches import PocketSwitches
from pocket_operator_session import prepare_manual, return_intent, run_intent
from types import SimpleNamespace


def button(index, value, initial=False):
    return dict(kind='button', index=index, value=value, initial=initial)


def test_held_on_attach_never_starts_and_requires_a_new_stable_press():
    edges = SwitchEdges([dict(kind='button', index=0, value=1, command='START')])
    edges.receive(button(0, 1, True), 0)
    assert edges.ready(1) == []
    edges.receive(button(0, 0), 2)
    edges.receive(button(0, 1), 3)
    assert edges.ready(3.1) == []
    assert edges.ready(3.2) == ['START']
    edges.receive(button(0, 1), 4)
    assert edges.ready(5) == []


def test_bounce_and_cancel_priority_do_not_queue_resume():
    edges = SwitchEdges([dict(kind='button', index=i, value=1, command=c)
                         for i, c in enumerate(('RESUME', 'CANCEL'))])
    for i in range(2):
        edges.receive(button(i, 0, True), 0)
    edges.receive(button(0, 1), 1)
    edges.receive(button(0, 0), 1.1)
    assert edges.ready(2) == []
    for i in range(2):
        edges.receive(button(i, 1), 3)
    assert edges.ready(3.2) == ['CANCEL']
    assert edges.ready(4) == []


def test_three_position_switch_uses_explicit_axis_values():
    edges = SwitchEdges([dict(kind='axis', index=4, value=v, command=c)
                         for v, c in ((-32767, 'PAUSE'), (32767, 'RESUME'))])
    edges.receive(dict(kind='axis', index=4, value=-32767, initial=True), 0)
    assert edges.ready(1) == []
    edges.receive(dict(kind='axis', index=4, value=32767, initial=False), 2)
    assert edges.ready(2.2) == ['RESUME']


@pytest.mark.parametrize('item', [dict(kind='button', index=0, value=1, command='ARM'),
    dict(kind='axis', index=-1, value=1, command='START'),
    dict(kind='button', index=0, value=0, command='START')])
def test_unsafe_or_invalid_mapping_is_rejected(item):
    with pytest.raises(ValueError):
        SwitchEdges([item])


def test_conflicting_mappings_are_rejected():
    with pytest.raises(ValueError):
        SwitchEdges([dict(kind='button', index=0, value=1, command=c) for c in ('START', 'CANCEL')])


def pocket_event(index, high, initial=False):
    return dict(kind='axis',index=index,value=32767 if high else -32767,initial=initial)


def test_pocket_sa_sb_sd_attach_high_does_not_arm_or_start():
    policy=PocketSwitches()
    for index in (4,5,7):policy.receive(pocket_event(index,True,True),0)
    assert policy.tick(1)==[]
    policy.receive(pocket_event(7,False),2)
    assert policy.tick(2.2)==['PAUSE']
    policy.receive(pocket_event(7,True),3)
    assert policy.tick(3.2)==['RUN']


def test_pocket_manual_or_arm_off_preempts_simultaneous_run():
    policy=PocketSwitches()
    for index in (4,5):policy.receive(pocket_event(index,True,True),0)
    policy.receive(pocket_event(7,False,True),0)
    policy.receive(pocket_event(5,False),1)
    policy.receive(pocket_event(7,True),1)
    assert policy.tick(1.2)==['MANUAL']
    policy.receive(pocket_event(4,False),2)
    assert policy.tick(2.2)==['DISARM']


def test_run_requires_both_sa_and_sb_and_never_retries_held_sd():
    policy=PocketSwitches()
    for index in (4,5,7):policy.receive(pocket_event(index,False,True),0)
    policy.receive(pocket_event(7,True),1)
    assert policy.tick(1.2)==['BLOCKED_RUN']
    policy.receive(pocket_event(4,True),2)
    policy.receive(pocket_event(5,True),2)
    assert policy.tick(2.2)==['ARM']
    assert policy.tick(3)==[]


@pytest.mark.parametrize('index', [0, 1])
def test_unassigned_key_and_latching_wheel_never_request_return(index):
    policy=PocketSwitches()
    for axis in (4,5,7):policy.receive(pocket_event(axis,True,True),0)
    policy.receive(button(index,1,True),0)
    assert policy.tick(1)==[]
    policy.receive(button(index,0),2)
    assert policy.tick(2.2)==[]
    policy.receive(button(index,1),3)
    assert policy.tick(3.2)==[]
    assert policy.tick(4)==[]
    policy.receive(button(index,0),5)
    policy.receive(pocket_event(7,False),5)
    assert policy.tick(5.2)==['PAUSE']


@pytest.mark.parametrize('fault', [None,'armed','airborne','navigation','active_task','no_qgc'])
def test_initial_manual_preparation_is_ground_only_and_never_arms(fault):
    calls=[]
    node=SimpleNamespace(samples={'vehicle_status':SimpleNamespace(arming_state=1),
        'vehicle_land_detected':SimpleNamespace(landed=True)},active_goal=None,
        operator_input_ready=lambda:True,operator_command=lambda command:(calls.append(command) or True,'REQUESTED'))
    switches=SimpleNamespace(navigation=False)
    if fault=='armed':node.samples['vehicle_status'].arming_state=2
    if fault=='airborne':node.samples['vehicle_land_detected'].landed=False
    if fault=='navigation':switches.navigation=True
    if fault=='active_task':node.active_goal=object()
    if fault=='no_qgc':node.operator_input_ready=lambda:False
    prepared=prepare_manual(node,switches,False)
    assert prepared==(fault is None)
    assert calls==(['MANUAL'] if fault is None else [])
    if prepared:
        assert prepare_manual(node,switches,prepared)
        assert calls==['MANUAL']


@pytest.mark.parametrize('kind,phase,override,expected', [
    ('NAVIGATE','NAVIGATE',False,'RETURN'),('RETURN','RETURN',False,'PAUSE'),
    ('RETURN','PAUSED',False,'RESUME'),('RETURN','PLAN_REQUEST',True,'PAUSE')])
def test_return_intent_pauses_both_recipe_return_and_requested_return(kind,phase,override,expected):
    node=SimpleNamespace(steps=[dict(type=kind)],step_index=0,phase=phase,return_override=override)
    assert return_intent(node)==expected


def test_explicit_momentary_button_zero_is_release_before_use_and_wheel_is_ignored():
    policy=PocketSwitches(return_button=0)
    for axis in (4,5,7):policy.receive(pocket_event(axis,True,True),0)
    policy.receive(button(0,1,True),0)
    assert policy.tick(1)==[]
    policy.receive(button(0,0),2)
    assert policy.tick(2.2)==[]
    policy.receive(button(0,1),3)
    policy.receive(button(1,1),3)
    assert policy.tick(3.2)==['TOGGLE_RETURN']
    assert policy.tick(4)==[]
    with pytest.raises(ValueError):PocketSwitches(return_button=1)


@pytest.mark.parametrize('phase,override,checkpoint,expected',[
    ('PAUSED',True,object(),'RESUME_MISSION'),('PAUSED',True,None,'RESUME'),
    ('PAUSED',False,None,'RESUME'),('RETURN',True,object(),None)])
def test_sd_resume_original_work_is_distinct_from_return_key(phase,override,checkpoint,expected):
    node=SimpleNamespace(phase=phase,return_override=override,return_checkpoint=checkpoint,
        step_index=3,steps=[{}, {}, {},dict(type='RETURN')])
    assert run_intent(node)==expected
    if phase=='PAUSED':
        assert return_intent(node)=='RESUME'


def test_finish_intent_preempts_simultaneous_resume_original_work():
    edges=SwitchEdges([dict(kind='button',index=i,value=1,command=command)
        for i,command in enumerate(('RESUME_MISSION','END_MISSION'))])
    for i in (0,1):edges.receive(button(i,0,True),0)
    for i in (0,1):edges.receive(button(i,1),1)
    assert edges.ready(1.2)==['END_MISSION']
