"""Build records accept byte-identical registered moves, reject changed inputs."""
import hashlib
import json

import pytest

import build_vio_sdk_candidate as builder


def test_registered_move_preserves_hash_and_rejects_content_changes(tmp_path, monkeypatch):
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    layout = tmp_path/'scripts/compat/layout.json'
    layout.parent.mkdir(parents=True)
    layout.write_text(json.dumps(dict(scripts={'scripts/old.hpp': 'scripts/sdk/current.hpp'}, resources={})))
    current = tmp_path/'scripts/sdk/current.hpp'
    current.parent.mkdir()
    current.write_text('reviewed bytes')
    digest = hashlib.sha256(current.read_bytes()).hexdigest()
    assert builder.canonical_input_hashes({'scripts/old.hpp': digest}) == {'scripts/sdk/current.hpp': digest}
    current.write_text('changed bytes')
    with pytest.raises(RuntimeError, match='input drift'):
        builder.canonical_input_hashes({'scripts/old.hpp': digest})


def test_unknown_missing_path_is_not_treated_as_a_move(tmp_path, monkeypatch):
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    layout = tmp_path/'scripts/compat/layout.json'
    layout.parent.mkdir(parents=True)
    layout.write_text('{"scripts":{},"resources":{}}')
    with pytest.raises(RuntimeError, match='input drift'):
        builder.canonical_input_hashes({'unknown.hpp': 'old digest'})
