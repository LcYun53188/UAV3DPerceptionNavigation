"""Read-only audit of the PX4 reference depth stream (OakD-Lite model, not OAK-D Pro W calibration)."""
import math
import numpy as np


class DepthAudit:
    def __init__(self, profile=None):
        self.profile = profile or dict(width=640, height=480, horizontal_fov_rad=1.274, near_m=.2, far_m=19.1)
        self.images = 0
        self.infos = 0
        self.valid_images = 0
        self.valid_infos = 0
        self.errors = []
        self.last_image = None
        self.last_info = None
        self.last_image_receive = None
        self.last_info_receive = None
        self.last_stamp = None
        self.regressions = 0

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def image(self, message, now):
        self.images += 1
        try:
            if message.encoding != '32FC1' or (message.width, message.height) != (self.profile['width'], self.profile['height']):
                raise ValueError('Unexpected depth encoding or dimensions')
            if message.step < message.width * 4 or len(message.data) != message.step * message.height:
                raise ValueError('Invalid depth buffer layout')
            if not message.header.frame_id:
                raise ValueError('Missing depth frame')
            dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
            values = np.ndarray((message.height, message.width), dtype=dtype,
                                buffer=bytes(message.data), strides=(message.step, 4))
            finite = values[np.isfinite(values)]
            if not len(finite) or np.any(finite < self.profile['near_m'] - 1e-5) or np.any(finite > self.profile['far_m'] + 1e-5):
                raise ValueError('No finite depth or depth outside pinned sensor clip')
            stamp = self.stamp(message)
            if stamp <= 0:
                raise ValueError('Invalid image timestamp')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.regressions += 1
                raise ValueError('Depth timestamp did not advance')
            self.last_stamp = stamp
            self.last_image = dict(stamp=stamp, frame=message.header.frame_id,
                                   width=message.width, height=message.height,
                                   encoding=message.encoding, finite_pixels=int(len(finite)),
                                   finite_fraction=float(len(finite) / values.size),
                                   min_m=float(finite.min()), max_m=float(finite.max()))
            self.last_image_receive = now
            self.valid_images += 1
        except (ValueError, TypeError, BufferError) as exc:
            if len(self.errors) < 10:
                self.errors.append(str(exc))

    def info(self, message, now):
        self.infos += 1
        width, height = self.profile['width'], self.profile['height']
        expected_focal = width / (2 * math.tan(self.profile['horizontal_fov_rad'] / 2))
        expected_k = [expected_focal, 0., width / 2, 0., expected_focal, height / 2, 0., 0., 1.]
        expected_p = [expected_focal, 0., width / 2, 0., 0., expected_focal, height / 2, 0., 0., 0., 1., 0.]
        expected_r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

        def matches(actual, expected):
            return len(actual) == len(expected) and all(
                math.isfinite(a) and abs(a - e) < 1e-4 for a, e in zip(actual, expected))

        valid = ((message.width, message.height) == (self.profile['width'], self.profile['height'])
                 and bool(message.header.frame_id) and self.stamp(message) > 0
                 and matches(message.k, expected_k) and matches(message.p, expected_p)
                 and matches(message.r, expected_r)
                 and len(message.d) in (0, 5) and all(math.isfinite(v) and abs(v) < 1e-8 for v in message.d))
        if valid:
            self.valid_infos += 1
            self.last_info_receive = now
            self.last_info = dict(stamp=self.stamp(message), frame=message.header.frame_id,
                                  width=message.width, height=message.height,
                                  k=list(message.k), p=list(message.p), r=list(message.r), d=list(message.d))
        elif len(self.errors) < 10:
            self.errors.append('CameraInfo differs from pinned depth calibration')

    def result(self, now, clock, writers):
        image_age = now - self.last_image_receive if self.last_image_receive is not None else None
        info_age = now - self.last_info_receive if self.last_info_receive is not None else None
        source_age = clock - self.last_image['stamp'] if clock is not None and self.last_image else None
        matching = bool(self.last_image and self.last_info
                        and self.last_image['frame'] == self.last_info['frame']
                        and abs(self.last_image['stamp'] - self.last_info['stamp']) < .5)
        passed = (self.valid_images >= 5 and self.valid_infos >= 5 and not self.errors
                  and matching and image_age < 1 and info_age < 1
                  and source_age is not None and -.05 <= source_age < 1
                  and writers == {'/px4_depth/image': 1, '/px4_depth/camera_info': 1})
        return dict(passed=passed, images=self.images, infos=self.infos,
                    valid_images=self.valid_images, valid_infos=self.valid_infos,
                    errors=self.errors, regressions=self.regressions,
                    image=self.last_image, camera_info=self.last_info,
                    receive_age_s=dict(image=image_age, camera_info=info_age),
                    image_source_age_s=source_age, matching=matching, writers=writers,
                    scope='disarmed sensor transport only; no TF, fusion, alignment or flight validation')
