#!/usr/bin/env python3
"""RGB detection / AR planning display with stereo previews; no flight publishers."""
import argparse
import json
import time
from collections import deque
from pathlib import Path
import tkinter as tk

import cv2
import numpy as np

from vision_overlay import TargetSelection, fresh, image_bgr, project_segments, stamp_seconds

BG, PANEL, TEXT, MUTED, ACCENT = '#0b121b', '#14212e', '#e5eff8', '#8ea3b7', '#4ae0c0'


class VisionMonitor:
    def __init__(self, root, out=None, *, demo=False):
        self.root, self.out, self.demo = root, out, demo
        self.node = None
        self.follow_client = None
        self.cache, self.images, self.rendered = {}, {}, {}
        self.detections = deque(maxlen=12)
        self.rgb_frames = deque(maxlen=12)
        self.selection = TargetSelection()
        self.display_boxes, self.display_serial = [], None
        self.display_time = 0.
        self.hitboxes = []
        self.context_key = None
        self.context_since = float('inf')
        self.path_context = None
        self.last_clock = None
        self.last_capture, self.capture_count = 0., 0
        self.image_serial = {}
        self.sdk_pose = self.px4_local = None
        self.last_saved_stamp = None
        self.ar_reason = '等待 RGB / 标定 / 规划路线'
        self.notice = '点击 RGB 人体框选择跟踪目标；右键或 Esc 清除选择'
        self.show_boxes, self.show_ar = tk.BooleanVar(value=True), tk.BooleanVar(value=True)
        root.title('UAV · 视觉与任务监控' + (' · 演示数据' if demo else ''))
        root.geometry('1400x1000')
        root.minsize(1050, 850)
        root.configure(bg=BG)
        root.protocol('WM_DELETE_WINDOW', self.close)
        header = tk.Frame(root, bg=BG)
        header.pack(fill='x', padx=22, pady=(16, 10))
        tk.Label(header, text='UAV  /  VISION', font=('sans', 21, 'bold'), fg=TEXT, bg=BG).pack(side='left')
        tk.Label(header, text='演示数据 · 无 ROS 连接' if demo else 'RGB · 人体检测 · 规划投影 · 双目',
                 fg=ACCENT, bg=BG, font=('sans', 11)).pack(side='right')
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=20)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        main = tk.Frame(body, bg=PANEL)
        main.grid(row=0, column=0, sticky='nsew', padx=(0, 14))
        self.caption(main, 'RGB  /  DETECTION + PLANNED ROUTE')
        self.rgb = tk.Canvas(main, bg='#080e15', highlightthickness=0)
        self.rgb.pack(fill='both', expand=True, padx=8)
        self.rgb.bind('<Button-1>', self.select)
        self.rgb.bind('<Button-3>', lambda _: self.clear_selection())
        root.bind('<Escape>', lambda _: self.clear_selection())
        self.rgb_meta = tk.Label(main, bg=PANEL, fg=MUTED, anchor='w', justify='left')
        self.rgb_meta.pack(fill='x', padx=14, pady=10)
        right = tk.Frame(body, bg=PANEL, width=330)
        right.grid(row=0, column=1, sticky='nsew')
        self.stereo = {}
        stereo_row = tk.Frame(right, bg=PANEL)
        stereo_row.pack(fill='x', padx=8)
        stereo_row.columnconfigure((0, 1), weight=1, uniform='stereo')
        for column, (side, title) in enumerate((('left', 'LEFT / 左目'), ('right', 'RIGHT / 右目'))):
            cell = tk.Frame(stereo_row, bg=PANEL)
            cell.grid(row=0, column=column, sticky='nsew', padx=3)
            self.caption(cell, title)
            canvas = tk.Canvas(cell, width=160, height=130, bg='#080e15', highlightthickness=0)
            canvas.pack(fill='x')
            self.stereo[side] = canvas
        self.caption(right, 'STATUS  /  实时状态')
        self.status = tk.Label(right, bg=PANEL, fg=TEXT, justify='left', anchor='nw',
                               wraplength=310, font=('sans', 10))
        self.status.pack(fill='x', padx=14)
        self.caption(right, 'CONTROLS  /  操作')
        for text, var in (('显示人体检测框', self.show_boxes), ('显示 AR 规划路线', self.show_ar)):
            tk.Checkbutton(right, text=text, variable=var, bg=PANEL, fg=TEXT,
                           selectcolor=BG, activebackground=PANEL, activeforeground=TEXT).pack(anchor='w', padx=10)
        row = tk.Frame(right, bg=PANEL)
        row.pack(fill='x', padx=12, pady=7)
        self.button(row, '保存截图', self.snapshot).pack(side='left', padx=(0, 8))
        self.button(row, '清除选择', self.clear_selection).pack(side='left')
        row = tk.Frame(right, bg=PANEL)
        row.pack(fill='x', padx=12)
        self.follow_buttons = {}
        for verb,label in (('START','开始预演'),('PAUSE','暂停预演'),('CANCEL','取消预演')):
            button = self.button(row,label,lambda verb=verb:self.follow_command(verb),state='disabled')
            button.pack(side='left',padx=(0,5))
            self.follow_buttons[verb] = button
        self.follow_hint = tk.Label(right,text='等待跟随后端',fg=MUTED,bg=PANEL,
                                    wraplength=310,anchor='w',justify='left')
        self.follow_hint.pack(fill='x',padx=14,pady=7)
        self.auto_button = self.button(right,'开启人物跟踪模式（预演）',self.toggle_auto,state='disabled')
        self.auto_button.pack(fill='x',padx=14,pady=5)
        self.auto_status_label = tk.Label(right,text='自动检测移动人物 · 限时记录轨迹',
            fg=MUTED,bg=PANEL,anchor='w',justify='left',wraplength=310)
        self.auto_status_label.pack(fill='x',padx=14,pady=5)
        self.footer = tk.Label(root, fg=MUTED, bg=BG, anchor='w', justify='left', wraplength=1300)
        self.footer.pack(fill='x', padx=22, pady=12)
        if not demo:
            self.connect_ros()
        self.root.after(100, self.tick)

    @staticmethod
    def caption(parent, title):
        tk.Label(parent, text=title, fg=MUTED, bg=PANEL, anchor='w',
                 font=('sans', 10, 'bold')).pack(fill='x', padx=14, pady=(12, 7))

    @staticmethod
    def button(parent, text, command=None, **kwargs):
        return tk.Button(parent, text=text, command=command, bg='#233b4c', fg=TEXT,
                         activebackground='#31546b', activeforeground=TEXT,
                         relief='flat', padx=8, pady=6, **kwargs)

    def put(self, key, message):
        self.cache[key] = (message, time.monotonic())

    def connect_ros(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo
        from vision_msgs.msg import Detection2DArray
        from nav_msgs.msg import Path as RosPath
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from uav_nav_interfaces.msg import PlanningContext, TimedTrajectory
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        self.node = rclpy.create_node('uav_vision_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.tf, self.node)
        from person_follow_client import PersonFollowClient
        self.follow_client = PersonFollowClient(self.node)
        topics = {'rgb': (Image, '/px4_rgb/image'), 'left': (Image, '/vio/left/image'),
                  'right': (Image, '/vio/right/image'), 'info': (CameraInfo, '/px4_depth/camera_info'),
                  'path': (RosPath, '/planning/ego/path'),
                  'trajectory': (TimedTrajectory, '/planning/ego/trajectory'),
                  'context': (PlanningContext, '/planning/context'),
                  'diagnostics': (String, '/uav/people/diagnostics')}
        for key, (kind, topic) in topics.items():
            # Latest only: the monitor never backlogs sensor frames or large paths.
            self.node.create_subscription(kind, topic, lambda msg, key=key: self.receive(key, msg),
                                          qos_profile_sensor_data)
        self.node.create_subscription(Detection2DArray, '/uav/people/detections',
                                      lambda msg: self.detections.append((msg, time.monotonic())), qos_profile_sensor_data)
        estimator = 'cuvslam'
        if self.out:
            try:
                estimator = json.loads((self.out/'manifest.json').read_text()).get('estimator', estimator)
            except (OSError, ValueError):
                pass
        self.node.create_subscription(PoseWithCovarianceStamped,
            '/openvins/pose' if estimator == 'openvins' else '/visual_slam/tracking/vo_pose_covariance',
            self.receive_pose, qos_profile_sensor_data)
        try:
            from px4_msgs.msg import VehicleLocalPosition
            version = getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0)
            self.node.create_subscription(VehicleLocalPosition,
                '/px4_7/fmu/out/vehicle_local_position'+('_v'+str(version) if version else ''),
                self.receive_position, qos_profile_sensor_data)
        except ImportError:
            self.notice = 'PX4 消息环境未加载；RGB / AR 显示仍可用'

    def receive_pose(self, message):
        self.sdk_pose = dict(stamp=stamp_seconds(message.header.stamp), z_variance=message.pose.covariance[14])

    def receive_position(self, message):
        self.px4_local = dict(px4_timestamp=message.timestamp, z=message.z, z_valid=message.z_valid)

    def receive(self, key, message):
        # Replayed stamps cannot renew receipt age or keep a frozen stream live.
        previous = self.cache.get(key)
        if (previous and hasattr(message, 'header')
                and stamp_seconds(message.header.stamp) <= stamp_seconds(previous[0].header.stamp)):
            if key == 'context' and not message.valid:
                self.selection.lose('地图或定位上下文失效')
                self.cache.pop('path',None)
                self.cache.pop('trajectory',None)
                self.cache[key] = (message,previous[1])
                self.context_key = None
            return
        if key == 'context':
            identity = (message.map_id, message.map_epoch, message.localization_session,
                        message.alignment_id, message.alignment_generation, message.context_id,
                        tuple(message.reset_counters))
            if not message.valid or identity != self.context_key:
                self.cache.pop('path', None)
                self.cache.pop('trajectory', None)
                self.context_since = stamp_seconds(message.header.stamp)
                self.selection.lose('地图或定位上下文改变')
                self.path_context = None
            self.context_key = identity if message.valid else None
        elif key == 'path':
            self.path_context = self.context_key
        self.put(key, message)
        if key == 'rgb':
            self.rgb_frames.append((message, time.monotonic()))

    def display_rgb(self):
        """Use a bounded recent source frame when detector inference arrives later."""
        now = self.node.get_clock().now().nanoseconds * 1e-9
        mono = time.monotonic()
        ordered = sorted(self.detections,key=lambda pair:stamp_seconds(pair[0].header.stamp),reverse=True)
        watermark = self.image_serial.get('rgb')
        for detection, received in ordered:
            if watermark and detection.header.frame_id == watermark[0] and stamp_seconds(detection.header.stamp)<watermark[1]:
                continue
            if not fresh(stamp_seconds(detection.header.stamp), now, received, mono):
                continue
            for rgb, arrival in reversed(self.rgb_frames):
                if (rgb.header.stamp == detection.header.stamp and rgb.header.frame_id == detection.header.frame_id
                        and fresh(stamp_seconds(rgb.header.stamp), now, arrival, mono)):
                    return rgb
        return self.current('rgb')

    def current(self, key, limit=1.):
        record = self.cache.get(key)
        if not record:
            return None
        msg, received = record
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return msg if fresh(stamp_seconds(msg.header.stamp), now, received, time.monotonic(), limit) else None

    def ar_segments(self, rgb):
        from rclpy.time import Time
        info, path, context = self.current('info'), self.current('path', 2.), self.current('context')
        trajectory = self.current('trajectory', 2.)
        if (context is None or not context.valid or not context.map_id or context.header.frame_id != 'map'
                or self.path_context != self.context_key or self.context_key is None):
            return [], '隐藏：规划上下文失效或改变'
        if info is None or path is None or trajectory is None:
            return [], '隐藏：缺少新鲜标定或配对规划路线'
        if (path.header.stamp != trajectory.header.stamp or trajectory.header.frame_id != 'map'
                or (trajectory.map_id, trajectory.epoch) != (context.map_id, context.map_epoch)
                or not trajectory.trajectory_id or trajectory.map_version > context.map_version
                or stamp_seconds(path.header.stamp) < self.context_since):
            return [], '隐藏：路线与轨迹 / 地图身份不匹配'
        if (info.header.frame_id != rgb.header.frame_id or path.header.frame_id != 'map'
                or (info.width, info.height) != (rgb.width, rgb.height)
                or any(abs(value) > 1e-9 for value in info.d)
                or not all(np.isfinite(info.d))
                or any(p.header.frame_id not in ('', 'map') for p in path.poses)):
            return [], '隐藏：相机尺寸 / 坐标系 / 畸变模型不支持'
        if len(path.poses) > 4096:
            return [], '隐藏：路线超过显示上限'
        try:
            transform = self.tf.lookup_transform(rgb.header.frame_id, 'map', Time.from_msg(rgb.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            segments = project_segments([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path.poses],
                                        (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), info.k, rgb.width, rgb.height)
            return segments, '规划参考路线 · 不代表执行器已接受' if segments else '规划路线位于视野之外'
        except Exception as error:
            # TF lookup failures must remove the previous overlay immediately.
            return [], '隐藏：历史 TF / 投影不可用 (' + type(error).__name__ + ')'

    def draw(self, canvas, image):
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        scale = min(width/image.shape[1], height/image.shape[0])
        w, h = max(1, int(image.shape[1]*scale)), max(1, int(image.shape[0]*scale))
        small = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
        data = f'P6\n{w} {h}\n255\n'.encode() + small[:, :, ::-1].tobytes()
        photo = tk.PhotoImage(data=data, format='PPM')
        canvas.delete('all')
        x, y = (width-w)//2, (height-h)//2
        canvas.create_image(x, y, image=photo, anchor='nw')
        canvas.photo = photo
        return scale, x, y

    def placeholder(self, canvas, text):
        canvas.delete('all')
        canvas.create_text(max(1, canvas.winfo_width())//2, max(1, canvas.winfo_height())//2,
                           text=text, fill=MUTED, font=('sans', 12))

    def select(self, event):
        if time.monotonic()-self.display_time > 1. or self.display_serial is None:
            self.notice = '当前画面已过期，请等待新鲜检测'
            return
        matches = [index for x1,y1,x2,y2,index in self.hitboxes
                   if x1 <= event.x <= x2 and y1 <= event.y <= y2]
        if len(matches) != 1:
            self.notice = '点击处存在重叠目标，请选择独立检测框' if matches else '请点击人体检测框；右键可清除选择'
            return
        self.selection.choose(self.display_boxes, matches[0], self.display_serial, self.display_time)
        self.notice = f'已选定目标 T{self.selection.sequence}；等待后端绑定三维人体观测'
        self.update()

    def clear_selection(self):
        self.selection.clear()
        self.notice = '已清除本地选择'
        if self.follow_client:
            self.follow_client.last_send = float('-inf')
            self.follow_client.send_selection(self.selection,None,None)

    def follow_command(self,verb):
        if not self.follow_client:
            self.notice = '演示模式没有 ROS 后端'
            return
        status = self.follow_client.current(self.selection.sequence)
        if verb == 'PAUSE' and status and status['phase'] == 'PAUSED':
            verb = 'RESUME'
        _,self.notice = self.follow_client.command(verb,self.selection.sequence)

    def toggle_auto(self):
        if not self.follow_client:
            return
        current = self.follow_client.auto_status()
        if current:
            _,self.notice = self.follow_client.set_auto(not current['enabled'])

    def snapshot(self):
        if 'rgb' not in self.rendered:
            self.notice = '没有可保存的新鲜 RGB 画面'
            return
        directory = (self.out or Path('.cache/vision-monitor')) / 'vision-snapshots'
        directory.mkdir(parents=True, exist_ok=True)
        prefix = directory / str(time.time_ns())
        for key, frame in self.rendered.items():
            if not cv2.imwrite(str(prefix) + '-' + key + '.png', frame):
                self.notice = '截图保存失败'
                return
        Path(str(prefix)+'.json').write_text(json.dumps(dict(demo=self.demo, ar=self.ar_reason,
            selected=self.selection.snapshot(), system_ns=time.time_ns()), ensure_ascii=False, indent=2)+'\n')
        self.notice = '截图已保存：' + str(prefix)

    def demo_frames(self):
        image = np.full((540, 960, 3), (34, 31, 26), np.uint8)
        cv2.rectangle(image, (60, 45), (900, 360), (75, 81, 88), 3)
        for x in range(-400, 1500, 160):
            cv2.line(image, (480, 260), (x, 540), (60, 66, 73), 1)
        for y in (320, 370, 440, 530):
            cv2.line(image, (0, y), (960, y), (60, 66, 73), 1)
        cv2.circle(image, (630, 210), 24, (137, 153, 169), -1)
        cv2.rectangle(image, (603, 240), (657, 360), (137, 153, 169), -1)
        for x in (615, 645):
            cv2.line(image, (x, 350), (x, 445), (137, 153, 169), 15)
        self.images['rgb'] = image
        mono = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.images['left'] = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        self.images['right'] = np.roll(self.images['left'], -12, axis=1)
        self.ar_reason = '演示投影 · 非实时规划'
        segments = project_segments([(-.8,.9,1.5),(-.4,.9,2.),(.2,.9,3.),(.8,.9,5.)],
                                    (0,0,0),(0,0,0,1),[580,0,480,0,580,270,0,0,1],960,540)
        return segments, [(590,175,675,455,'demo 0.94')]

    def capture_stereo(self):
        """Retain the existing bounded timestamped stereo diagnostic evidence."""
        if self.out is None or self.capture_count >= 240 or time.monotonic()-self.last_capture < 1:
            return
        left, right = self.current('left'), self.current('right')
        if (left is None or right is None or left.header.stamp != right.header.stamp
                or left.header.stamp == self.last_saved_stamp):
            return
        directory = self.out / 'flight-images'
        directory.mkdir(exist_ok=True)
        self.capture_count += 1
        self.last_capture = time.monotonic()
        self.last_saved_stamp = left.header.stamp
        name = f'{self.capture_count:04d}'
        for side in ('left', 'right'):
            cv2.imwrite(str(directory/(name+'-'+side+'.ppm')), self.images[side])
        (directory/(name+'.json')).write_text(json.dumps(dict(system_ns=time.time_ns(),
            mono=self.last_capture, image_stamp=stamp_seconds(left.header.stamp),
            sdk_pose=self.sdk_pose, px4_local=self.px4_local), indent=2)+'\n')

    def tick(self):
        try:
            self.update()
        except Exception as error:
            self.rendered.clear()
            self.hitboxes = []
            self.selection.lose('显示异常')
            for canvas in (self.rgb, *self.stereo.values()):
                self.placeholder(canvas, '显示异常，等待下一帧')
            self.notice = '显示错误：' + type(error).__name__ + ': ' + str(error)[:160]
            self.footer.configure(text=self.notice)
        self.root.after(100, self.tick)

    def update(self):
        segments, boxes = [], []
        serial = None
        self.images, self.rendered = {}, {}
        if self.demo:
            segments, boxes = self.demo_frames()
            serial = ('demo', time.monotonic())
        else:
            import rclpy
            deadline = time.monotonic() + .02
            for _ in range(30):
                rclpy.spin_once(self.node, timeout_sec=0)
                if time.monotonic() >= deadline:
                    break
            now = self.node.get_clock().now().nanoseconds * 1e-9
            if self.last_clock is not None and now < self.last_clock:
                self.cache.clear()
                self.detections.clear()
                self.rgb_frames.clear()
                self.context_key = self.path_context = None
                self.selection.lose('时钟回退')
                self.tf.clear()
            self.last_clock = now
            for key in ('rgb', 'left', 'right'):
                msg = self.current(key)
                if msg is not None:
                    self.images[key] = image_bgr(msg)
            rgb = self.display_rgb()
            if rgb is not None:
                self.images['rgb'] = image_bgr(rgb)
                serial = (rgb.header.frame_id, stamp_seconds(rgb.header.stamp))
                self.image_serial['rgb'] = serial
                segments, self.ar_reason = self.ar_segments(rgb)
                for detection, received in reversed(self.detections):
                    if (detection.header.stamp != rgb.header.stamp or detection.header.frame_id != rgb.header.frame_id
                            or not fresh(stamp_seconds(detection.header.stamp), now, received, time.monotonic())):
                        continue
                    for item in detection.detections:
                        b = item.bbox
                        score = max((r.hypothesis.score for r in item.results), default=0.)
                        values = (b.center.position.x, b.center.position.y, b.size_x, b.size_y, score)
                        if not all(np.isfinite(values)) or b.size_x <= 0 or b.size_y <= 0:
                            continue
                        boxes.append((int(b.center.position.x-b.size_x/2), int(b.center.position.y-b.size_y/2),
                                      int(b.center.position.x+b.size_x/2), int(b.center.position.y+b.size_y/2),
                                      f'person {score:.2f}'))
                    break
            else:
                self.ar_reason = '隐藏：RGB 缺失或过期'
            if 'left' in self.images and 'right' in self.images:
                self.capture_stereo()
        if not boxes:
            serial = None
        if serial != self.display_serial:
            self.display_time = time.monotonic()
        self.display_boxes, self.display_serial = boxes, serial
        self.selection.update(boxes, serial, time.monotonic())
        follow = None
        if self.follow_client:
            self.follow_client.send_selection(self.selection,serial,self.context_key)
            follow = self.follow_client.current(self.selection.sequence)
        self.hitboxes = []
        for key, canvas in (('rgb', self.rgb), *self.stereo.items()):
            if key not in self.images:
                self.placeholder(canvas, '等待新鲜 RGB 图像' if key == 'rgb' else '等待新鲜双目图像')
                continue
            frame = self.images[key].copy()
            if key == 'rgb':
                if self.show_ar.get():
                    for start, end in segments:
                        cv2.line(frame, start, end, (204, 219, 46), 4, cv2.LINE_AA)
                if self.show_boxes.get():
                    for index, (x1,y1,x2,y2,label) in enumerate(boxes):
                        selected = self.selection.state == 'SELECTED' and self.selection.index == index
                        color = (45,192,255) if selected else (160,224,74)
                        if selected:
                            label = f'T{self.selection.sequence} SELECTED / ' + label
                        cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                        cv2.putText(frame, label, (x1,max(20,y1-9)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
                if self.demo:
                    cv2.putText(frame, 'DEMO / SYNTHETIC DATA', (25,35), cv2.FONT_HERSHEY_SIMPLEX,.7,(45,192,255),2)
            self.rendered[key] = frame
            scale, x, y = self.draw(canvas, frame)
            if key == 'rgb' and self.show_boxes.get():
                self.hitboxes = [(x+a*scale,y+b*scale,x+c*scale,y+d*scale,i)
                                 for i,(a,b,c,d,_) in enumerate(boxes)]
        self.rgb_meta.configure(text=f'检测框 {len(boxes)}  ·  AR: {self.ar_reason}\n'
                                      '点击人体框选择跟踪目标 · 黄色：选定目标 · 右键 / Esc：清除')
        context_text = '演示状态' if self.demo else '定位 / 地图：等待新鲜上下文'
        stereo_text = '双目：演示'
        if not self.demo:
            context = self.current('context')
            if context is not None:
                context_text = f'定位 / 地图：{context.reason}\n地图：{context.map_id[:18]} / {context.map_epoch}'
            left, right = self.current('left'), self.current('right')
            stereo_text = ('双目：同时间戳' if left.header.stamp == right.header.stamp else '双目：时间戳不一致') if left and right else '双目：缺失 / 过期'
        self.status.configure(text=f'{context_text}\n{stereo_text}\n目标：'+
            (f'T{self.selection.sequence} · {self.selection.reason}' if self.selection.state != 'NONE' else '未选择')+
            '\n控制权：仅观察'+('\n跟随预演：'+follow['phase']+'\n'+follow['reason'] if follow else '\n跟随后端：未连接 / 等待选择绑定'))
        if follow and follow.get('candidate_goal') is not None:
            goal = follow['candidate_goal']
            self.status.configure(text=self.status.cget('text')+f'\n候选点：{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f} m\n目标运动：{follow["motion"]}')
        for verb,button in self.follow_buttons.items():
            allowed = bool(follow and not self.follow_client.pending and
                           ((verb == 'START' and follow['phase'] == 'READY') or
                            (verb == 'PAUSE' and follow['phase'] in ('PREVIEW','PAUSED')) or
                            (verb == 'CANCEL' and follow['phase'] in ('PREVIEW','PAUSED'))))
            button.configure(state='normal' if allowed else 'disabled')
        self.follow_buttons['PAUSE'].configure(text='继续预演' if follow and follow['phase'] == 'PAUSED' else '暂停预演')
        self.follow_hint.configure(text='预演仅生成候选点，不发送飞行目标。\n执行阻断：动态安全未验收；PX4 行为树跟随步骤待接入。' if follow else
                                  ('演示模式：任务按钮不连接 ROS' if self.demo else '等待后端确认当前选择与三维观测'))
        automatic = self.follow_client.auto_status() if self.follow_client else None
        self.auto_button.configure(state='normal' if automatic and not self.follow_client.auto_pending else 'disabled',
            text='关闭人物跟踪模式' if automatic and automatic['enabled'] else '开启人物跟踪模式（预演）')
        self.auto_status_label.configure(text=(f"自动模式：{automatic['phase']}\n{automatic['reason']}\n轨迹记录：{automatic['episode_count']} 段\n原任务继续；当前仅记录和预演"+
                                              ('\n记录失败：'+automatic['recording_error'] if automatic.get('recording_error') else '') if automatic else
                                              '自动模式：后端不可用' if not self.demo else '演示模式：未连接自动跟踪后端'))
        terminal = ''
        if self.out and not self.demo:
            try:
                from simulation.readiness import startup_status, status_text
            except ImportError:
                terminal = '等待任务状态；启动准入由现有任务控制系统检查'
            else:
                terminal = status_text(startup_status(self.out))
            for filename in ('flight-observation.json', 'ui-session.json'):
                try:
                    receipt = json.loads((self.out/filename).read_text())
                    result = receipt.get('result', {})
                    terminal += (f"\n任务：{result.get('code', receipt.get('state', 'UNKNOWN'))} "
                                 f"{result.get('reason', '')}；清理确认：{result.get('cleanup_confirmed', False)}")
                except (OSError, ValueError, AttributeError):
                    pass
        command_notice = self.follow_client.notice if self.follow_client else ''
        self.footer.configure(text=self.notice + ('\n'+command_notice if command_notice else '') + ('\n'+terminal if terminal else ''))

    def close(self):
        if self.node is not None:
            self.clear_selection()
            self.node.destroy_node()
            self.node = None
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', nargs='?', type=Path)
    parser.add_argument('--demo', action='store_true', help='Synthetic display only; no ROS connection')
    parser.add_argument('--close-after', type=float, default=0, help='Close this viewer after N seconds')
    args = parser.parse_args(argv)
    if args.run_directory and not args.run_directory.is_dir():
        parser.error('run_directory must be an existing directory')
    if not args.demo:
        import rclpy
        rclpy.init()
    root = tk.Tk()
    try:
        monitor = VisionMonitor(root, args.run_directory, demo=args.demo)
        if args.close_after > 0:
            root.after(int(args.close_after*1000), monitor.close)
        root.mainloop()
    finally:
        if not args.demo:
            rclpy.try_shutdown()
