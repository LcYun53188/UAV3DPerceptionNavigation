#!/usr/bin/env python3
"""Desktop RadioMaster Pocket UAV panel: AETR channels and optional uinput output."""
import argparse
import json
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog

from pocket_panel_model import CHANNELS, PocketModel
from pocket_uav_switches import PocketUavSwitches
from pocket_virtual_joystick import NAME, VirtualJoystick

BG, CARD, INK, MUTED, ACCENT = '#121820', '#1d2733', '#edf2f7', '#99aabd', '#ffad55'


class Stick(tk.Canvas):
    def __init__(self, parent, model, horizontal, vertical, label, throttle=False):
        super().__init__(parent, width=240, height=260, bg=CARD, highlightthickness=0)
        self.model, self.horizontal, self.vertical, self.throttle = model, horizontal, vertical, throttle
        self.create_text(120, 17, text=label, fill=INK, font=('sans', 12, 'bold'))
        self.create_oval(25, 40, 215, 230, fill=BG, outline='#405066', width=3)
        self.create_line(120, 43, 120, 228, fill='#344152')
        self.create_line(28, 135, 212, 135, fill='#344152')
        self.knob = self.create_oval(0, 0, 0, 0, fill=ACCENT, outline='#ffcd92', width=2)
        self.bind('<ButtonPress-1>', self.move)
        self.bind('<B1-Motion>', self.move)
        self.bind('<ButtonRelease-1>', self.release)

    def move(self, event):
        self.model.set_channel(self.horizontal, (event.x - 120) / 90)
        self.model.set_channel(self.vertical, (135 - event.y) / 90)
        self.draw()

    def release(self, _=None):
        self.model.set_channel(self.horizontal, 0.)
        if not self.throttle:
            self.model.set_channel(self.vertical, 0.)
        self.draw()

    def draw(self):
        x = 120 + self.model.values[self.horizontal - 1] * 90
        y = 135 - self.model.values[self.vertical - 1] * 90
        self.coords(self.knob, x - 13, y - 13, x + 13, y + 13)


class PocketPanel:
    def __init__(self, root, *, connect=False):
        self.root, self.model, self.device = root, PocketModel(), None
        self.intent_preview = PocketUavSwitches()
        self.preview_levels = {}
        self.last_intent = '等待控件边沿；预览不代表飞控已执行'
        self.pressed = set()
        self.key_releases = {}
        self.mouse_se_down = False
        self.last_tick = time.monotonic()
        self.closed = False
        self.after_id = None
        root.title('RadioMaster Pocket · UAV / Model 5 · 电脑仿真')
        root.configure(bg=BG)
        root.geometry('1080x920+60+60')
        root.minsize(1020, 900)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<KeyPress>', self.key_press)
        root.bind('<KeyRelease>', self.key_release)
        root.bind('<FocusOut>', lambda _: root.after_idle(self.focus_check))
        # SE must release even when the pointer leaves its button.
        root.bind('<ButtonRelease-1>', self.mouse_release, add='+')
        self.heading = tk.Label(root, text='POCKET  /  UAV', fg=INK, bg=BG,
                                font=('sans', 23, 'bold'), anchor='w')
        self.heading.pack(fill='x', padx=26, pady=(18, 3))
        tk.Label(root, text='MODEL 5    ·    MODE 2 / AETR    ·    CH1—CH16',
                 fg=MUTED, bg=BG, anchor='w', font=('sans', 10)).pack(fill='x', padx=28)
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=24, pady=16)
        controls = tk.Frame(body, bg=CARD, padx=14, pady=12)
        controls.pack(side='left', fill='both', expand=True)
        monitor = tk.Frame(body, bg=CARD, padx=14, pady=12)
        monitor.pack(side='right', fill='both', padx=(16, 0))
        top = tk.Frame(controls, bg=CARD)
        top.pack(fill='x')
        self.switch_buttons = {}
        self.make_switch(top, 5, 'SA · L1', ['上锁', '解锁'], [-1, 1], 0)
        self.make_switch(top, 8, 'SD · R1', ['正常', '返航'], [-1, 1], 1)
        middle = tk.Frame(controls, bg=CARD)
        middle.pack(fill='x', pady=12)
        self.make_switch(middle, 6, 'SB · L2 / 主模式', ['手动 −1', '定点 0', '自动 +1'], [-1, 0, 1], 0)
        self.make_switch(middle, 7, 'SC · R2 / 随主模式切换', self.model.sc_labels, [-1, 0, 1], 1)
        sticks = tk.Frame(controls, bg=CARD)
        sticks.pack(pady=(5, 0))
        self.left = Stick(sticks, self.model, 4, 3, '左摇杆 · 偏航 / 油门', throttle=True)
        self.right = Stick(sticks, self.model, 1, 2, '右摇杆 · 横滚 / 俯仰')
        self.left.pack(side='left', padx=5)
        self.right.pack(side='left', padx=5)
        tk.Label(controls, text='油门保持位置；其余轴松开回中', fg=MUTED, bg=CARD).pack()
        rear = tk.Frame(controls, bg=CARD)
        rear.pack(fill='x', pady=(14, 8))
        self.se = self.button(rear, 'SE · LB  回弹键\n短按暂停 / 继续 · 长按取消')
        self.se.pack(side='left', padx=(0, 12))
        self.se.bind('<ButtonPress-1>', lambda _: self.press_se())
        tk.Label(rear, text='S1 · RB  速度通道', bg=CARD, fg=INK).pack(anchor='w')
        self.speed = tk.Scale(rear, from_=-100, to=100, orient='horizontal', length=225,
            bg=CARD, fg=INK, troughcolor=BG, highlightthickness=0, activebackground=ACCENT,
            command=lambda value: self.model.set_channel(9, float(value) / 100))
        self.speed.set(-100)
        self.speed.pack()
        self.gesture = tk.Label(controls, text='', fg=ACCENT, bg=CARD, wraplength=500)
        self.gesture.pack(fill='x', pady=5)
        self.mode_hint = tk.Label(controls, fg=INK, bg=CARD, wraplength=610, justify='left', anchor='w')
        self.mode_hint.pack(fill='x', pady=3)
        actions = tk.Frame(controls, bg=CARD)
        actions.pack(fill='x', pady=9)
        self.button(actions, '导出通道 JSON', self.export).pack(side='left')
        tk.Label(monitor, text='CHANNEL MONITOR', bg=CARD, fg=INK,
                 font=('sans', 12, 'bold')).pack(anchor='w', pady=(0, 10))
        self.rows = []
        for index in range(16):
            name = CHANNELS[index][0] if index < len(CHANNELS) else 'Reserved'
            row = tk.Frame(monitor, bg=CARD)
            row.pack(fill='x', pady=2)
            tk.Label(row, text=f'CH{index+1:02}  {name}', width=15, anchor='w',
                     bg=CARD, fg=INK, font=('monospace', 10)).pack(side='left')
            meter = tk.Canvas(row, width=100, height=23, bg=BG, highlightthickness=0)
            meter.pack(side='left', padx=6)
            meter.create_line(50, 2, 50, 21, fill=MUTED)
            bar = meter.create_rectangle(50, 7, 50, 16, fill=ACCENT, outline='')
            label = tk.Label(row, width=6, bg=CARD, fg=INK, font=('monospace', 10))
            label.pack(side='left')
            self.rows.append((meter, bar, label))
        tk.Label(monitor, text='CH10 / L01：SE 短按脉冲\nCH11 / L02：SE 长按脉冲\nCH12—CH16：预留，输出 0%',
                 bg=CARD, fg=MUTED, justify='left').pack(anchor='w', pady=15)
        tk.Label(monitor, text='SA / SB / SC / SD：F1 / F2 / F3 / F4\nSE：按住空格\n'
                 '油门 W/S · 偏航 A/D\n横滚 ←/→ · 俯仰 ↑/↓\n\nS1 速度请求待任务后端绑定',
                 bg=CARD, fg=MUTED, justify='left', font=('sans', 9)).pack(anchor='w')
        bottom = tk.Frame(root, bg=BG)
        bottom.pack(side='bottom', before=body, fill='x', padx=24, pady=(0, 16))
        self.connection_button = self.button(bottom, '连接虚拟摇杆', self.toggle_device)
        self.connection_button.pack(side='right', padx=(12, 0))
        self.status = tk.Label(bottom, text='本地预览 · 尚未连接仿真输入', fg=MUTED,
            bg=BG, anchor='w', justify='left', wraplength=820)
        self.status.pack(side='left', fill='x', expand=True)
        if connect:
            self.toggle_device()
        self.tick()

    @staticmethod
    def button(parent, text, command=None):
        return tk.Button(parent, text=text, command=command, bg='#304052', fg=INK,
                         activebackground=ACCENT, activeforeground=BG, relief='flat',
                         padx=12, pady=8, cursor='hand2', takefocus=False)

    def make_switch(self, parent, channel, label, choices, values, column):
        parent.grid_columnconfigure(column, weight=1)
        frame = tk.Frame(parent, bg=CARD)
        frame.grid(row=0, column=column, sticky='ew', padx=5)
        tk.Label(frame, text=label, bg=CARD, fg=MUTED, anchor='w').pack(fill='x', pady=(0, 5))
        buttons = []
        group = tk.Frame(frame, bg=CARD)
        group.pack(anchor='w')
        for title, value in zip(choices, values):
            button = self.button(group, title, lambda c=channel, v=value: self.model.set_channel(c, v))
            button.pack(side='left', padx=(0, 3))
            buttons.append((button, value))
        self.switch_buttons[channel] = buttons

    def press_se(self):
        self.mouse_se_down = True
        self.model.press_se(time.monotonic())

    def mouse_release(self, _):
        # Mouse and keyboard can overlap; release only when neither holds SE.
        self.mouse_se_down = False
        if 'space' not in self.pressed:
            self.model.release_se(time.monotonic())

    def key_press(self, event):
        key = event.keysym.lower()
        if key in self.key_releases:
            self.root.after_cancel(self.key_releases.pop(key))
        if key in self.pressed:
            return
        self.pressed.add(key)
        if key == 'space':
            self.model.press_se(time.monotonic())
            return 'break'
        if key in ('f1', 'f2', 'f3', 'f4'):
            channel = {'f1': 5, 'f2': 6, 'f3': 7, 'f4': 8}[key]
            choices = [-1, 0, 1] if channel in (6, 7) else [-1, 1]
            self.model.set_channel(channel, choices[(choices.index(self.model.values[channel - 1]) + 1) % len(choices)])
            return 'break'

    def key_release(self, event):
        # Linux Tk repeat emits a release/press pair. Cancel the delayed release
        # on the next press so holding Space cannot turn into repeated short taps.
        key = event.keysym.lower()
        if key in self.key_releases:
            self.root.after_cancel(self.key_releases.pop(key))
        self.key_releases[key] = self.root.after(15, lambda: self.finish_key_release(key))

    def finish_key_release(self, key):
        self.key_releases.pop(key, None)
        self.pressed.discard(key)
        if key == 'space' and not self.mouse_se_down:
            self.model.release_se(time.monotonic())

    def focus_check(self):
        if self.closed:
            return
        if self.root.focus_displayof() is None:
            self.pressed.clear()
            self.mouse_se_down = False
            self.model.release_controls()

    def export(self):
        path = filedialog.asksaveasfilename(parent=self.root, title='导出 UAV 通道',
            defaultextension='.json', initialfile='pocket-uav-channels.json',
            filetypes=[('JSON', '*.json')])
        if path:
            Path(path).write_text(json.dumps(self.model.snapshot(time.monotonic()),
                                            ensure_ascii=False, indent=2) + '\n')

    def toggle_device(self):
        if self.device is not None:
            self.device.close()
            self.device = None
            self.connection_button.configure(text='连接虚拟摇杆')
            self.status.configure(text='虚拟摇杆已断开 · 本地预览', fg=MUTED)
            return
        if self.model.values[4] != -1 or self.model.values[7] != -1 or self.model.values[2] != -1:
            self.status.configure(text='连接前请将 SA 上锁、SD 正常、油门拉到底', fg=ACCENT)
            return
        try:
            self.device = VirtualJoystick()
            self.device.write(self.model.tick(time.monotonic()))
        except OSError as error:
            if self.device is not None:
                self.device.close()
                self.device = None
            self.status.configure(text=f'虚拟摇杆连接失败：{error}；可继续本地预览', fg=ACCENT)
            return
        self.connection_button.configure(text='断开虚拟摇杆')
        print(json.dumps(dict(name=NAME, profile='uav-aetr-v1', scope='virtual input only')), flush=True)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(.1, now - self.last_tick), now
        for channel, negative, positive in ((1, 'left', 'right'), (2, 'down', 'up'), (4, 'a', 'd')):
            # Mouse still owns a stick until a keyboard axis has been used.
            if negative in self.pressed or positive in self.pressed or channel in getattr(self, 'keyboard_axes', set()):
                if not hasattr(self, 'keyboard_axes'):
                    self.keyboard_axes = set()
                self.keyboard_axes.add(channel)
                self.model.set_channel(channel, .65 * (int(positive in self.pressed) - int(negative in self.pressed)))
                if negative not in self.pressed and positive not in self.pressed:
                    self.keyboard_axes.discard(channel)
        self.model.set_channel(3, self.model.values[2] + dt * .8 * (int('w' in self.pressed) - int('s' in self.pressed)))
        channels = self.model.tick(now)
        for axis in (4, 5, 6, 7):
            self.preview_event('axis', axis, round(channels[axis]*32767), now)
        for button, channel in ((0, 9), (1, 10)):
            self.preview_event('button', button, int(channels[channel] > 0), now)
        commands = self.intent_preview.tick(now)
        if commands:
            self.last_intent = '意图预览：' + ' / '.join(commands)
            print(json.dumps(dict(preview_only=True, commands=commands)), flush=True)
        visual=(tuple(channels),self.model.se_since is not None,self.model.last_gesture,self.last_intent)
        if visual != getattr(self,'last_visual',None):
            self.last_visual=visual
            for (meter, bar, label), value in zip(self.rows, channels):
                meter.coords(bar, min(50, 50 + value * 47), 7, max(50, 50 + value * 47), 16)
                label.configure(text=f'{value*100:+.0f}%')
            for channel, buttons in self.switch_buttons.items():
                for index, (button, value) in enumerate(buttons):
                    selected = channels[channel - 1] == value
                    button.configure(bg=ACCENT if selected else '#304052', fg=BG if selected else INK)
                    if channel == 7:
                        button.configure(text=self.model.sc_labels[index], state='disabled' if self.model.main_mode == 'POSITION' else 'normal')
            hint = {'MANUAL': f'手动请求：{self.model.requested_mode}；SC 选择控制 / 定位方式',
                    'POSITION': '定点请求：VIO 保持接入；SC 档位不改变定位源',
                    'AUTO': f'自动请求：{self.model.sc_labels[int(channels[6])+1]}；需先在准备档通过准入'}[self.model.main_mode]
            landing = {'AUTO':'避障返航并降落','MANUAL':'原地缓慢降落','POSITION':'原地缓慢降落'}[self.model.main_mode]
            requirement={'MANUAL':'原地慢降需有效的局部位置与高度',
                         'AUTO':'返航使用 PX4 Home；需定位、地图和避障导航就绪',
                         'POSITION':'原地慢降需有效的局部位置与高度'}[self.model.main_mode]
            self.mode_hint.configure(text=hint + f'\nSD 自锁：按下 → {landing}；弹起复位\n' + requirement+'\n'+self.last_intent)
            self.switch_buttons[8][1][0].configure(text=landing)
            self.left.draw()
            self.right.draw()
            self.se.configure(bg=ACCENT if self.model.se_since is not None else '#304052')
            self.gesture.configure(text=self.model.last_gesture)
        if self.device is not None:
            try:
                self.device.write(channels)
                path = self.device.device_path()
                readable = path is not None and os.access(path, os.R_OK)
                device_status = path if readable else '等待 joydev / udev 用户读取权限'
                status_text=f'{NAME} · {device_status}\n在 QGC 选择此设备、校准并启用；任务请求需运行 UAV 配置控制器'
                if status_text != getattr(self,'last_status_text',None):
                    self.last_status_text=status_text
                    self.status.configure(text=status_text,fg=ACCENT)
                if readable and path != getattr(self, 'last_device_path', None):
                    self.last_device_path = path
                    print(json.dumps(dict(device=path, profile='uav-aetr-v1')), flush=True)
            except OSError as error:
                self.toggle_device()
                self.status.configure(text=f'虚拟输入已停止：{error}', fg=ACCENT)
        self.after_id = self.root.after(20, self.tick)

    def preview_event(self, kind, index, value, now):
        key = kind, index
        if self.preview_levels.get(key) != value:
            self.intent_preview.receive(dict(kind=kind, index=index, value=value,
                initial=key not in self.preview_levels), now)
            self.preview_levels[key] = value

    def close(self):
        self.closed = True
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        if self.device is not None:
            self.device.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uinput', action='store_true', help='Connect a Linux virtual joystick at startup')
    args = parser.parse_args()
    root = tk.Tk()
    PocketPanel(root, connect=args.uinput)
    root.mainloop()


if __name__ == '__main__':
    main()
