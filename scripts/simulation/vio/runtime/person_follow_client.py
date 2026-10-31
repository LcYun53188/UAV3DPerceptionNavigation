"""Nonblocking UI client for selection/preview; execution readiness comes from backend."""
import json
import math
import time
import uuid

from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_nav_interfaces.srv import SetMissionIntent

from vision_overlay import fresh

PREFIX = '/uav/people/follow'


class PersonFollowClient:
    def __init__(self,node):
        self.node = node
        self.client_id = str(uuid.uuid4())
        self.sequence = 0
        self.last_send = float('-inf')
        self.status = None
        self.received = 0.
        self.pending = None
        self.notice = ''
        self.publisher = node.create_publisher(String,PREFIX+'/selection',10)
        self.subscription = node.create_subscription(String,PREFIX+'/status',self.receive,10)
        self.service = node.create_client(SetMissionIntent,PREFIX+'/set_intent')
        self.auto_service = node.create_client(SetBool,PREFIX+'/auto_mode')
        self.auto_pending = None

    def receive(self,msg):
        try:
            if len(msg.data)>16384:
                return
            value = json.loads(msg.data)
            if not isinstance(value,dict) or not all(isinstance(value.get(key),str) for key in ('coordinator_instance','mission_uuid')):
                return
            uuid.UUID(value['coordinator_instance'])
            uuid.UUID(value['mission_uuid'])
            if (value['schema'] != 1 or value['mode'] != 'PREVIEW_ONLY' or value['flight_execution'] is not False
                    or type(value['sequence']) is not int
                    or type(value['source_stamp']) not in (int,float) or not math.isfinite(value['source_stamp'])
                    or value['phase'] not in ('IDLE','ACQUIRE','READY','PREVIEW','PAUSED','LOST')
                    or not isinstance(value['reason'],str)):
                return
            goal = value.get('candidate_goal')
            if goal is not None and (not isinstance(goal,list) or len(goal) != 3
                    or not all(type(v) in (int,float) and math.isfinite(v) for v in goal)):
                return
            automatic = value.get('automatic')
            if automatic is not None and (not isinstance(automatic,dict)
                    or type(automatic.get('enabled')) is not bool
                    or not isinstance(automatic.get('phase'),str)
                    or not isinstance(automatic.get('reason'),str)
                    or type(automatic.get('episode_count')) is not int):
                return
            previous = self.status
            if (previous and value['coordinator_instance'] == previous['coordinator_instance']
                    and value['sequence'] <= previous['sequence']):
                return
            self.status,self.received = value,time.monotonic()
        except (KeyError,ValueError,TypeError):
            return

    def current(self,selection_id=None):
        value = self.status
        now = self.node.get_clock().now().nanoseconds*1e-9
        if (value and value.get('client_id') == self.client_id
                and (selection_id is None or value.get('selection_id') == selection_id)
                and fresh(value.get('source_stamp',float('nan')),now,self.received,time.monotonic(),.75)
                and len(self.node.get_publishers_info_by_topic(PREFIX+'/status')) == 1):
            return value
        return None

    def auto_status(self):
        value = self.status
        now = self.node.get_clock().now().nanoseconds*1e-9
        if (value and fresh(value.get('source_stamp',float('nan')),now,self.received,time.monotonic(),.75)
                and len(self.node.get_publishers_info_by_topic(PREFIX+'/status')) == 1):
            return value.get('automatic')
        return None

    def set_auto(self,value):
        if self.auto_pending or self.auto_status() is None or not self.auto_service.service_is_ready():
            return False,'自动模式后端未就绪或正在等待回复'
        self.auto_pending = (self.auto_service.call_async(SetBool.Request(data=value)),time.monotonic())
        return True,'模式请求已发送，等待后端确认'

    def send_selection(self,selection,serial,context):
        mono = time.monotonic()
        if mono-self.last_send < .2:
            return
        self.last_send = mono
        self.sequence += 1
        payload = dict(schema=1,client_id=self.client_id,sequence=self.sequence,
                       selection_id=max(1,selection.sequence),valid=selection.state == 'SELECTED' and serial is not None)
        if payload['valid']:
            payload.update(box=list(selection.box),source_stamp=serial[1],frame_id=serial[0],
                           context=list(context[:6]) if context is not None else None)
        self.publisher.publish(String(data=json.dumps(payload,allow_nan=False)))
        self.poll()

    def command(self,verb,selection_id):
        status = self.current(selection_id)
        if self.pending:
            return False,'等待上一条请求的回复'
        if status is None or not self.service.service_is_ready():
            return False,'跟随后端不可用或状态已过期'
        request = SetMissionIntent.Request()
        request.command = verb
        request.mission_uuid.uuid = list(uuid.UUID(status['mission_uuid']).bytes)
        request.coordinator_instance = status['coordinator_instance']
        request.request_id.uuid = list(uuid.uuid4().bytes)
        self.pending = (self.service.call_async(request),time.monotonic(),
                        status['coordinator_instance'],status['mission_uuid'])
        return True,'请求已发送，等待后端确认'

    def poll(self):
        if self.auto_pending:
            future,started = self.auto_pending
            if future.done():
                self.auto_pending = None
                try:
                    response = future.result()
                    self.notice = ('模式已更新：' if response.success else '模式请求拒绝：')+response.message
                except Exception:
                    self.notice = '模式请求失败，以实时状态为准'
            elif time.monotonic()-started>2.:
                self.auto_pending = None
                self.notice = '模式请求超时，以实时状态为准'
        if not self.pending:
            return
        future,started,instance,session = self.pending
        if future.done():
            self.pending = None
            if not self.status or (instance,session) != (self.status['coordinator_instance'],self.status['mission_uuid']):
                self.notice = '忽略旧跟随会话的迟到回复'
                return
            try:
                response = future.result()
                self.notice = ('后端已接受：' if response.accepted else '后端拒绝：') + response.reason
            except Exception:
                self.notice = '后端请求失败；以实时状态为准'
        elif time.monotonic()-started>2.:
            self.pending = None
            self.notice = '请求回复超时；未确认执行，以实时状态为准'
