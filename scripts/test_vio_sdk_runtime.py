"""A requested queue depth alone must not establish the SDK runtime config."""
from pathlib import Path
import sys
from types import SimpleNamespace
from rclpy.parameter import Parameter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_sdk_runtime import SdkRuntimeReceipt


def receipt(values):
    obj=SdkRuntimeReceipt.__new__(SdkRuntimeReceipt)
    obj.expected={'image_qos_depth':1};obj.actual=None;obj.error=None;obj.cancelled=False
    obj.future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(values=values))
    obj.timer=SimpleNamespace(cancel=lambda:setattr(obj,'cancelled',True))
    return obj


def test_actual_depth_mismatch_cannot_qualify():
    obj=receipt([Parameter('depth',value=10).get_parameter_value()]);obj.tick()
    assert obj.actual=={'image_qos_depth':10} and not obj.result()['passed'] and obj.cancelled


def test_actual_matching_depth_is_receipted():
    obj=receipt([Parameter('depth',value=1).get_parameter_value()]);obj.tick()
    assert obj.result()['passed'] and obj.cancelled


def test_missing_service_value_cannot_be_silently_zipped():
    obj=receipt([]);obj.tick()
    assert obj.error=='SDK_PARAMETER_COUNT' and not obj.result()['passed'] and obj.cancelled
