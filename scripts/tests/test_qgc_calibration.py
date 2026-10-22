"""Retain real QGC calibration and verify signed throttle endpoint conversion."""
import configparser

import pytest

import qgc_calibration as calibration
from pocket_input import configure_qgc_input


def identity(virtual=False):
    return dict(name='Pocket Virtual UAV AETR' if virtual else 'OpenTX Radiomaster Pocket Joystick',
                axes=9 if virtual else 8, buttons=3 if virtual else 24,
                input_profile='uav-aetr-v1' if virtual else 'pocket-rc-v2')


def calibrated(device, center=-32768):
    config = configparser.ConfigParser(interpolation=None)
    config.optionxform = str
    prefix = calibration.device_prefix(device)
    values = {prefix+'calibrated':'true'}
    for axis,function in enumerate((0,1,3,2)):
        base = prefix+f'JoystickAxisSettingsArray\\{axis}\\'
        values.update({base+'function':str(function),base+'min':'-32768',base+'max':'32767',
            base+'center':str(center if function==3 else 0),base+'deadband':'13106' if function==3 else '0',
            base+'reversed':'false'})
    config['JoystickSettingsV2'] = values
    config['JoystickManager'] = {'joystickEnabledVehiclesIds':'8','activeJoystickName':device['name']}
    return config


def upstream_px4_throttle(raw, center):
    # QGC 5.1.5 down-zero mode ignores deadband for the throttle function.
    # This is its independent range model, then MAVLink z -> PX4 [-1,1].
    if center == -32768:
        adjusted = max(0,raw-center)/(32767-center)
    else:
        adjusted = (raw-center)/(32767-center) if raw>center else (raw-center)/(center+32768)
    wire_z = int((adjusted+1)/2*1000)
    return wire_z/500-1


def test_repair_retains_calibration_and_endpoints_but_restores_full_native_range():
    device = identity()
    config = calibrated(device)
    prefix = calibration.device_prefix(device)
    before = dict(config['JoystickSettingsV2'])
    correction = calibration.normalize_throttle(config,device)
    assert correction == dict(axis=2,previous_center=-32768,center=0,min=-32768,max=32767)
    after = dict(config['JoystickSettingsV2'])
    key = prefix+'JoystickAxisSettingsArray\\2\\center'
    assert {k for k in before if before[k]!=after[k]} == {key}
    assert calibration.ready(config,device)
    assert upstream_px4_throttle(-32768,-32768) == 0.
    assert [upstream_px4_throttle(raw,0) for raw in (-32768,0,32767)] == [-1.,0.,1.]


@pytest.mark.parametrize('case', ['uncalibrated','missing_axis','centered','center_zero','virtual','custom_range'])
def test_unknown_or_intentional_calibration_is_not_overwritten(case):
    device = identity(case=='virtual')
    config = calibrated(device)
    prefix = calibration.device_prefix(device)
    values = config['JoystickSettingsV2']
    if case=='uncalibrated':values[prefix+'calibrated']='false'
    if case=='missing_axis':del values[prefix+'JoystickAxisSettingsArray\\1\\function']
    if case=='centered':values[prefix+'JoystickAxisSettingsArray\\2\\center']='0'
    if case=='center_zero':values[prefix+'throttleModeCenterZero']='true'
    if case=='custom_range':values[prefix+'JoystickAxisSettingsArray\\2\\min']='-25000'
    before = dict(values)
    assert calibration.normalize_throttle(config,device) is None
    assert dict(values)==before


def test_next_session_uses_separate_valid_device_cache(tmp_path,monkeypatch):
    monkeypatch.setattr(calibration,'CACHE',tmp_path/'saved')
    source = tmp_path/'original.ini'
    device = identity()
    with source.open('w') as stream:calibrated(device).write(stream)
    original = source.read_bytes()
    saved = calibration.remember(source,device)
    assert source.read_bytes()==original
    assert calibration.cached(device)==saved
    assert calibration.cached(identity(True)) is None
    loaded = calibration.read(saved)
    assert loaded['JoystickManager']['joystickEnabledVehiclesIds']=='8'
    assert loaded['JoystickSettingsV2'][calibration.device_prefix(device)+'JoystickAxisSettingsArray\\2\\center']=='0'


def test_incomplete_calibration_does_not_replace_saved_one(tmp_path,monkeypatch):
    monkeypatch.setattr(calibration,'CACHE',tmp_path/'saved')
    source = tmp_path/'original.ini'
    device = identity()
    with source.open('w') as stream:calibrated(device).write(stream)
    saved = calibration.remember(source,device)
    good = saved.read_bytes()
    source.write_text('[JoystickSettingsV2]\n')
    assert calibration.remember(source,device) is None
    assert saved.read_bytes()==good


def test_startup_also_repairs_an_explicit_completed_qgc_config(tmp_path):
    device = identity()
    source = tmp_path/'qgc.ini'
    with source.open('w') as stream:calibrated(device).write(stream)
    configure_qgc_input(source,device)
    config = calibration.read(source)
    assert config['JoystickSettingsV2'][calibration.device_prefix(device)+'JoystickAxisSettingsArray\\2\\center']=='0'
    assert config['JoystickManager']['activeJoystickName']==device['name']


def test_arm_rejection_is_printed_once_with_the_actual_input(tmp_path,capsys):
    import json
    from simulation.session import report_rejected_arm
    (tmp_path/'pocket-live.json').write_text(json.dumps(dict(last_operator_request=dict(
        command='ARM',mono=10.,accepted=False,reason='ARM_THROTTLE_NOT_LOW',manual_control={'throttle':0.}))))
    last = report_rejected_arm(tmp_path,None)
    assert last == 10.
    assert 'PX4 油门=0.0' in capsys.readouterr().out
    assert report_rejected_arm(tmp_path,last) == last
    assert capsys.readouterr().out == ''
