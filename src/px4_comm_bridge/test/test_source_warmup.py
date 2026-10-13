from types import SimpleNamespace
import pytest
from px4_comm_bridge.source_warmup import SourceWarmup


def stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


@pytest.mark.parametrize('start', [10_000_000_000, 1_791_540_000_000_000_001])
def test_warmup_uses_original_time_with_exact_epoch_boundary(start):
    gate = SourceWarmup(20.)
    assert not gate.ready(stamp(start))
    assert not gate.ready(stamp(start + 19_999_999_999))
    # Repeated input cannot complete warmup while the simulator is paused.
    assert not gate.ready(stamp(start + 19_999_999_999))
    assert gate.ready(stamp(start + 20_000_000_000))


def test_startup_rollback_restarts_warmup_and_zero_duration_is_compatible():
    gate = SourceWarmup(20.)
    assert not gate.ready(stamp(30_000_000_000))
    assert not gate.ready(stamp(10_000_000_000))
    assert not gate.ready(stamp(29_999_999_999))
    assert gate.ready(stamp(30_000_000_000))
    assert SourceWarmup().ready(stamp(1))


@pytest.mark.parametrize('duration', [-1., float('nan'), float('inf')])
def test_invalid_warmup_is_rejected(duration):
    with pytest.raises(ValueError):
        SourceWarmup(duration)
