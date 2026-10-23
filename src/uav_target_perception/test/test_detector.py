import hashlib

import numpy as np
import pytest

from uav_target_perception import detector


def raw_output():
    data = np.zeros((1, 3549, 85), dtype=np.float32)
    # Anchor (10,10) at stride 8: centre (80,80), width/height 32.
    data[0, 530, :4] = [0, 0, np.log(4), np.log(4)]
    data[0, 530, 4:6] = [.9, .9]
    return data


def test_raw_stride_decoding_and_non_square_letterbox_inverse():
    result = detector.decode_people(raw_output(), (208, 416, 3))
    assert len(result) == 1
    assert np.allclose(result[0][0], [64, 64, 96, 96])
    assert result[0][1] == pytest.approx(.81)


def test_person_filter_rejects_other_classes_even_if_person_score_is_high():
    data = raw_output()
    data[0, 530, 6] = .95
    assert not detector.decode_people(data, (416, 416, 3))


def test_duplicate_person_boxes_are_suppressed():
    data = raw_output()
    data[0, 531] = data[0, 530]
    data[0, 531, 0] = -1
    assert len(detector.decode_people(data, (416, 416, 3))) == 1


def test_decoded_or_wrong_size_models_fail_instead_of_silent_bad_boxes():
    with pytest.raises(ValueError, match='raw YOLOX'):
        detector.decode_people(np.zeros((1, 8400, 85)), (416, 416, 3))


def test_prepare_checks_offline_content_and_preserves_existing_files(tmp_path, monkeypatch):
    source, target = tmp_path/'source.onnx', tmp_path/'model.onnx'
    source.write_bytes(b'known fixture')
    monkeypatch.setattr(detector, 'MODEL_SHA256', hashlib.sha256(source.read_bytes()).hexdigest())
    assert detector.prepare_model(target, source) == target
    assert detector.prepare_model(target) == target
    target.write_bytes(b'operator supplied model')
    with pytest.raises(ValueError, match='SHA-256'):
        detector.prepare_model(target, source)
    assert target.read_bytes() == b'operator supplied model'


def test_bad_download_is_removed_without_leaving_a_partial_model(tmp_path):
    source, target = tmp_path/'source.onnx', tmp_path/'model.onnx'
    source.write_bytes(b'incorrect content')
    with pytest.raises(ValueError):
        detector.prepare_model(target, source)
    assert not target.exists() and not list(tmp_path.glob('.person-*'))
