"""Read-only SDK parameter receipt; never changes estimator parameters."""
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
from rclpy.clock import Clock, ClockType


class SdkRuntimeReceipt:
    def __init__(self,node,expected):
        self.expected=dict(expected)
        self.actual=None
        self.error=None
        self.client=AsyncParameterClient(node,'/visual_slam')
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
