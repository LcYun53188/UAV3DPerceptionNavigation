"""Read-only SDK parameter receipt; never changes estimator parameters."""
import time
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
from rclpy.clock import Clock, ClockType


def wait_for_sdk_subscriptions(node, runtime_receipt, require_processes, timeout=45.):
    """Load the SDK before spawning PX4 so actual settling samples reach VIO."""
    import rclpy
    topics = ('/vio/left/image', '/vio/right/image', '/vio/imu')
    started = time.monotonic()
    while time.monotonic()-started < timeout:
        require_processes('SDK before PX4 spawn')
        endpoints = {topic: [e.node_name for e in node.get_subscriptions_info_by_topic(topic)]
                     for topic in topics}
        if runtime_receipt.result()['passed'] and all('visual_slam' in names for names in endpoints.values()):
            return dict(ready=True, wait_seconds=time.monotonic()-started,
                        subscriptions=endpoints, parameters=runtime_receipt.result(),
                        scope='SDK loaded before aircraft spawn; no motion or flight commands')
        rclpy.spin_once(node, timeout_sec=.05)
    raise RuntimeError('SDK subscriptions not ready before PX4 spawn; inspect vio.log')


class SdkRuntimeReceipt:
    def __init__(self,node,expected,node_name="/visual_slam"):
        self.expected=dict(expected)
        self.actual=None
        self.error=None
        self.client=AsyncParameterClient(node,node_name)
        self.future=None
        self.timer=node.create_timer(.5,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        if self.future is None and self.client.services_are_ready():
            self.future=self.client.get_parameters(list(self.expected))
        if self.future is not None and self.future.done():
            try:
                response=self.future.result()
                if len(response.values)!=len(self.expected):raise ValueError('SDK_PARAMETER_COUNT')
                self.actual=dict(zip(self.expected,map(parameter_value_to_python,response.values)))
            except Exception as exc:self.error=str(exc)
            self.timer.cancel()

    def result(self):
        return dict(schema=1,passed=self.actual==self.expected and self.error is None,
            expected=self.expected,actual=self.actual,error=self.error,
            scope='SDK parameter service snapshot; configured subscription depth, not queue occupancy')
