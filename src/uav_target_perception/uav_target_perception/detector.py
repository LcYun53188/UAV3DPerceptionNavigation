"""CPU YOLOX-Tiny person detector and checksum-verified model preparation.

Contract follows the official ONNXRuntime demo: raw stride 8/16/32 output,
416x416 float BGR input, top-left letterbox padding 114, no normalization.
https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import urllib.request

import numpy as np

MODEL_URL = ('https://github.com/Megvii-BaseDetection/YOLOX/releases/download/'
             '0.1.1rc0/yolox_tiny.onnx')
MODEL_SHA256 = '427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7'


def verify_model(path):
    path = Path(path).expanduser()
    if not path.is_file():
        raise ValueError(f'Missing person model: {path}; run person_model prepare first')
    if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError('Unsupported person model: SHA-256 differs from pinned YOLOX-Tiny release')
    return path


def prepare_model(destination, source=None):
    """Install from official release or an offline file; never replace a model."""
    destination = Path(destination).expanduser()
    if destination.exists():
        return verify_model(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, prefix='.person-', delete=False) as output:
            temporary = Path(output.name)
            reader = Path(source).expanduser().open('rb') if source else urllib.request.urlopen(MODEL_URL, timeout=30)
            with reader:
                while chunk := reader.read(1024*1024):
                    output.write(chunk)
            output.flush()
        verify_model(temporary)
        os.link(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return destination


def decode_people(output, image_shape, threshold=0.5, nms_threshold=0.45):
    cells, strides = [], []
    for stride in (8, 16, 32):
        size = 416//stride
        rows, cols = np.indices((size, size))
        cells.append(np.column_stack((cols.ravel(), rows.ravel())))
        strides.extend([stride]*(size*size))
    data = np.asarray(output)
    if data.shape != (1, len(strides), 85):
        raise ValueError(f'Expected raw YOLOX-Tiny [1,3549,85], got {data.shape}')
    if not 0 < threshold <= 1 or not 0 < nms_threshold < 1:
        raise ValueError('Invalid detector thresholds')
    data = data[0]
    classes = data[:, 5:].argmax(axis=1)
    scores = data[:, 4]*data[:, 5]
    indices = np.flatnonzero((classes == 0) & (scores >= threshold) & (scores <= 1)
                             & np.isfinite(data).all(axis=1))
    if not len(indices):
        return []
    stride = np.asarray(strides)[indices, None]
    centres = (data[indices, :2]+np.concatenate(cells)[indices])*stride
    sizes = np.exp(np.clip(data[indices, 2:4], -20, 20))*stride
    ratio = min(416/image_shape[0], 416/image_shape[1])
    boxes = np.concatenate((centres-sizes/2, centres+sizes/2), axis=1)/ratio
    boxes[:, (0, 2)] = boxes[:, (0, 2)].clip(0, image_shape[1])
    boxes[:, (1, 3)] = boxes[:, (1, 3)].clip(0, image_shape[0])
    order = np.argsort(scores[indices])[::-1][:300]
    detections = []
    while len(order) and len(detections) < 50:
        current, rest = order[0], order[1:]
        box = boxes[current]
        if np.any(box[2:]-box[:2] <= 1):
            order = rest
            continue
        detections.append((box.copy(), float(scores[indices[current]])))
        overlap = np.maximum(0, np.minimum(box[2:], boxes[rest, 2:])
                             - np.maximum(box[:2], boxes[rest, :2])).prod(axis=1)
        area = (box[2:]-box[:2]).prod()
        other_area = np.maximum(0, boxes[rest, 2:]-boxes[rest, :2]).prod(axis=1)
        iou = overlap/np.maximum(area+other_area-overlap, 1e-9)
        order = rest[iou <= nms_threshold]
    return detections


def merge_rotated_detections(groups, image_shape, nms_threshold=.45):
    """Map right-angle detections to the original pixel grid before cross-view NMS."""
    height, width = image_shape[:2]
    merged = []
    for angle, detections in groups:
        for box, score in detections:
            x1, y1, x2, y2 = map(float, box)
            if angle == 90:
                box = [y1, height-x2, y2, height-x1]
            elif angle == 270:
                box = [width-y2, x1, width-y1, x2]
            elif angle == 0:
                box = [x1, y1, x2, y2]
            else:
                raise ValueError('Unsupported detection rotation')
            box = np.asarray(box)
            box[[0, 2]] = box[[0, 2]].clip(0, width)
            box[[1, 3]] = box[[1, 3]].clip(0, height)
            merged.append((box, score))
    selected = []
    for box, score in sorted(merged, key=lambda item: item[1], reverse=True):
        duplicate = False
        for previous, _ in selected:
            intersection = np.maximum(0, np.minimum(box[2:], previous[2:])-np.maximum(box[:2], previous[:2])).prod()
            union = (box[2:]-box[:2]).prod()+(previous[2:]-previous[:2]).prod()-intersection
            if intersection/max(union, 1e-9) > nms_threshold:
                duplicate = True
                break
        if duplicate:
            continue
        selected.append((box, score))
        if len(selected) >= 50:
            break
    return selected


class PersonDetector:
    def __init__(self, model_path, threshold=0.5, threads=2, rotation_augmentation=False):
        import cv2
        if not 0 < threshold <= 1 or not 1 <= threads <= 8:
            raise ValueError('Invalid detector threshold or CPU thread count')
        path = verify_model(model_path)
        cv2.setNumThreads(threads)
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.threshold = threshold
        self.rotation_augmentation = rotation_augmentation

    def detect(self, bgr):
        normal = self._detect_upright(bgr)
        if not self.rotation_augmentation:
            return normal
        import cv2
        groups = [(0, normal), (90, self._detect_upright(cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE))),
                  (270, self._detect_upright(cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)))]
        return merge_rotated_detections(groups, bgr.shape)

    def _detect_upright(self, bgr):
        import cv2
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8 or not bgr.size:
            raise ValueError('Expected a nonempty uint8 BGR image')
        height, width = bgr.shape[:2]
        ratio = min(416/height, 416/width)
        resized = cv2.resize(bgr, (max(1, int(width*ratio)), max(1, int(height*ratio))))
        canvas = np.full((416, 416, 3), 114, dtype=np.uint8)
        canvas[:resized.shape[0], :resized.shape[1]] = resized
        self.net.setInput(cv2.dnn.blobFromImage(canvas, scalefactor=1., swapRB=False))
        return decode_people(self.net.forward(), bgr.shape, self.threshold)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('prepare', help='Verify/download the pinned official model')
    prepare.add_argument('destination', type=Path)
    prepare.add_argument('--source', type=Path, help='Offline copy; must match the release checksum')
    detect = sub.add_parser('detect', help='Run actual person inference on a local image')
    detect.add_argument('model', type=Path)
    detect.add_argument('image', type=Path)
    detect.add_argument('--threshold', type=float, default=0.5)
    detect.add_argument('--output', type=Path, help='Optional new annotated image')
    options = parser.parse_args(args)
    if options.command == 'prepare':
        path = prepare_model(options.destination, options.source)
        print(f'{MODEL_SHA256}  {path}')
        return
    import cv2
    image = cv2.imread(str(options.image))
    if image is None:
        parser.error('Cannot read the input image')
    detector = PersonDetector(options.model, options.threshold)
    started = time.monotonic()
    detections = detector.detect(image)
    result = dict(model_sha256=MODEL_SHA256, seconds=time.monotonic()-started,
                  people=[dict(box=box.tolist(), confidence=score) for box, score in detections])
    if options.output:
        for box, confidence in detections:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(image, f'person {confidence:.2f}', (x1, max(18, y1-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 200, 0), 2)
        success, encoded = cv2.imencode(options.output.suffix, image)
        if not success:
            parser.error('Unsupported output image format')
        options.output.parent.mkdir(parents=True, exist_ok=True)
        with options.output.open('xb') as handle:
            handle.write(encoded.tobytes())
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
