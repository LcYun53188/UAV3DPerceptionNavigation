"""Pure geometry and freshness checks for the camera operator display."""
import math
import numpy as np


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(source, now, received, mono, limit=1.0):
    return (all(math.isfinite(v) for v in (source, now, received, mono))
            and -.05 <= now - source <= limit and 0 <= mono - received <= limit)


def image_bgr(message):
    """Decode supported ROS images, respecting row padding and channel order."""
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    count = channels.get(message.encoding)
    if (count is None or message.width <= 0 or message.height <= 0
            or message.step < message.width * count
            or len(message.data) != message.step * message.height):
        raise ValueError('Unsupported or malformed image')
    pixels = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * count].reshape(message.height, message.width, count)
    if count == 1:
        return np.repeat(pixels, 3, axis=2)
    if message.encoding in ('rgb8', 'rgba8'):
        return pixels[:, :, :3][:, :, ::-1].copy()
    return pixels[:, :, :3].copy()


def project_segments(points, translation, quaternion, intrinsic, width, height):
    """Project map polyline using camera<-map TF, with near-plane/image clipping.

    Only rectified/pinhole calibration is supported by the caller. Camera axes
    must match CameraInfo: +x right, +y down, +z forward.
    """
    import cv2
    p = np.asarray(points, dtype=float)
    k = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    if p.size == 0:
        return []
    if (p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all()
            or q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all()
            or not np.isfinite(t).all() or not np.isfinite(k).all()
            or abs(np.linalg.norm(q) - 1) > .01 or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or width <= 0 or height <= 0):
        raise ValueError('Invalid projection geometry')
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    camera = p @ rotation.T + t
    result = []
    near = .05
    for first, second in zip(camera[:-1], camera[1:]):
        a, b = first.copy(), second.copy()
        if a[2] < near and b[2] < near:
            continue
        if a[2] < near:
            a += (b-a) * ((near-a[2])/(b[2]-a[2]))
        if b[2] < near:
            b += (a-b) * ((near-b[2])/(a[2]-b[2]))
        uv = np.array([k @ a, k @ b])
        uv = uv[:, :2] / uv[:, 2, None]
        # Bound conversion for clipLine's signed integer input.
        uv = np.clip(np.rint(uv), -10000000, 10000000).astype(int)
        visible, start, end = cv2.clipLine((0, 0, int(width), int(height)), tuple(uv[0]), tuple(uv[1]))
        if visible:
            result.append((start, end))
    return result


class TargetSelection:
    """Conservative local 2-D association, never a person identity or flight command."""
    def __init__(self):
        self.clear()
        self.sequence = 0

    def clear(self):
        self.state, self.reason = 'NONE', '未选择'
        self.index = self.box = self.serial = None
        self.others = []
        self.seen = None

    @staticmethod
    def overlap(first, second):
        x1, y1 = max(first[0], second[0]), max(first[1], second[1])
        x2, y2 = min(first[2], second[2]), min(first[3], second[3])
        area = max(0, x2-x1)*max(0, y2-y1)
        total = ((first[2]-first[0])*(first[3]-first[1])
                 + (second[2]-second[0])*(second[3]-second[1])-area)
        return area/total if total > 0 else 0.

    def choose(self, boxes, index, serial, now):
        self.sequence += 1
        self.state, self.reason = 'SELECTED', '目标已选定 · 图像关联'
        self.index, self.box, self.serial, self.seen = index, boxes[index][:4], serial, now
        self.others = [b[:4] for i,b in enumerate(boxes) if i != index]

    def lose(self, reason):
        if self.state == 'SELECTED':
            self.state, self.reason, self.index = 'LOST', reason+'；请重新选择', None

    def update(self, boxes, serial, now):
        if self.state != 'SELECTED':
            return
        if now-self.seen > 1. or now < self.seen:
            self.lose('目标观测超时')
            return
        if serial is None:
            self.lose('没有匹配检测')
            return
        if serial[0] != self.serial[0] or serial[1] < self.serial[1]:
            self.lose('图像坐标系或时间改变')
            return
        if serial == self.serial:
            return  # Repaint/replay cannot extend the association deadline.
        candidates = [i for i,b in enumerate(boxes) if self.overlap(self.box,b) >= .3]
        if len(candidates) != 1:
            self.lose('目标丢失' if not candidates else '多个候选存在歧义')
            return
        index = candidates[0]
        if any(self.overlap(other, boxes[index]) >= .3 for other in self.others):
            self.lose('目标交叉或遮挡，关联不确定')
            return
        self.index, self.box, self.serial, self.seen = index, boxes[index][:4], serial, now
        self.others = [b[:4] for i,b in enumerate(boxes) if i != index]

    def snapshot(self):
        return dict(state=self.state, reason=self.reason, selection_id=self.sequence,
                    box=self.box, source=self.serial)
