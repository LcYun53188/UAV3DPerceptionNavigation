"""Callbacks must retire a bound source before a newer healthy callback can hide loss."""
from collections import OrderedDict
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from uav_nav_interfaces.msg import VioStatus
from px4_msgs.msg import VehicleOdometry
sys.path.insert(0,str(Path(__file__).resolve().parent))
from px4_vio_real_fusion import RealPoseFusionAudit,json_message


def fixture():
    a=object.__new__(RealPoseFusionAudit)
    a.stream=SimpleNamespace(fault='',identity=('session','a'*64,0,'01','01'))
    def reject(reason):a.stream.fault=a.stream.fault or reason
    a.stream.reject=reject
    a.node=SimpleNamespace(get_publishers_info_by_topic=lambda _:[SimpleNamespace(endpoint_gid=bytes([1]))])
    from px4_comm_bridge.source_timing import SourceTiming
    a.timing=SourceTiming();a.ros=lambda:12.
    a.gid_modes=set()
    a.statuses=OrderedDict();a.pending=OrderedDict();a.drain=lambda:None
    m=VioStatus(localization_session='session',calibration_id='a'*64,valid=True)
    info=SimpleNamespace(publisher_gid=bytes([1]))
    return a,m,info


@pytest.mark.parametrize('fault',['tracking','reset','session','calibration','gid','duplicate_writer'])
def test_transient_source_fault_cannot_be_hidden_by_later_healthy_callback(fault):
    a,m,info=fixture()
    if fault=='tracking':m.valid=False;m.reason='VIO_TRACKING_INVALID'
    if fault=='reset':m.reset_counter=1
    if fault=='session':m.localization_session='replacement'
    if fault=='calibration':m.calibration_id='b'*64
    if fault=='gid':info.publisher_gid=bytes([2])
    if fault=='duplicate_writer':a.node.get_publishers_info_by_topic=lambda _:[object(),object()]
    a.on_status(m,info)
    assert a.stream.fault and not a.statuses
    a.on_status(VioStatus(localization_session='session',calibration_id='a'*64,valid=True),info)
    assert a.stream.fault and not a.statuses


def test_missing_pair_is_queued_without_emission_and_duplicate_fails():
    a,_,info=fixture();m=PoseWithCovarianceStamped()
    m.header.stamp.sec=12
    a.on_pose(m,info)
    assert len(a.pending)==1 and not a.stream.fault
    a.on_pose(m,info)
    assert a.stream.fault=='VIO_TIME_DISCONTINUITY'


def test_graph_gid_is_compared_with_actual_callback_gid():
    a,_,info=fixture();info.publisher_gid=bytes([2])
    a.on_pose(PoseWithCovarianceStamped(),info)
    assert a.stream.fault=='VIO_PUBLISHER_CHANGED' and not a.pending


def test_evidence_encodes_unknown_velocity_without_fabricating_zero():
    m=VehicleOdometry(velocity=[float('nan')]*3,velocity_variance=[float('nan')]*3)
    d=json_message(m)
    assert d['velocity']==[None]*3 and d['velocity_variance']==[None]*3


def test_dict_callback_info_from_local_rclpy_is_checked():
    a,m,_=fixture()
    a.on_status(m,{'publisher_gid':bytes([1])})
    assert not a.stream.fault and len(a.statuses)==1
    a.on_status(m,{'publisher_gid':bytes([2])})
    assert a.stream.fault=='VIO_PUBLISHER_CHANGED'


def test_missing_callback_gid_is_explicitly_reported_as_graph_only():
    a,m,_=fixture();a.on_status(m,{})
    assert not a.stream.fault and len(a.statuses)==1
    assert a.gid_modes=={'unique_graph_only'}


def test_full_telemetry_serialization_is_deferred_until_after_callbacks():
    a=object.__new__(RealPoseFusionAudit)
    m=VehicleOdometry(velocity=[float('nan')]*3,velocity_variance=[float('nan')]*3)
    a.history=[dict(mono=12.,name='ev',message=m)]
    a.echoes=[dict(mono=12.,message=m)]
    a.outputs=[dict(mono=12.,aligned=PoseWithCovarianceStamped(),ev=m)]
    evidence=a.evidence()
    assert a.history[0]['message'] is m and a.echoes[0]['message'] is m
    assert evidence['fusion-telemetry.json'][0]['message']['velocity']==[None]*3
    assert evidence['fusion-outputs.json'][0]['ev']==evidence['fusion-dds-echoes.json'][0]['message']
