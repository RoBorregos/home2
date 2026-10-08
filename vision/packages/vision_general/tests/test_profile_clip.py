"""Check timing boundaries and memory accounting without a GPU."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


path = (
    Path(__file__).resolve().parents[3] / "benchmarks/embedding_gallery/profile_clip.py"
)
spec = importlib.util.spec_from_file_location("profile_clip", path)
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


def test_measure_waits_for_gpu_before_and_after_each_call(monkeypatch):
    events = []
    torch = SimpleNamespace(
        cuda=SimpleNamespace(synchronize=lambda: events.append("sync"))
    )
    ticks = iter([1.0, 1.01, 2.0, 2.03])
    monkeypatch.setattr(profile.time, "perf_counter", lambda: next(ticks))
    result = profile.measure(torch, lambda: events.append("infer"), 2)
    assert events == ["sync", "infer", "sync", "sync", "infer", "sync"]
    assert result["samples_ms"] == pytest.approx([10, 30])
    assert result["mean_ms"] == pytest.approx(20)
    assert result["p95_ms"] == pytest.approx(29)


def test_memory_keeps_global_and_torch_counters_separate(monkeypatch):
    monkeypatch.setattr(profile.Path, "read_text", lambda _: "MemAvailable: 100 kB\n")
    cuda = SimpleNamespace(
        synchronize=Mock(),
        mem_get_info=lambda: (200, 1000),
        memory_allocated=lambda: 10,
        memory_reserved=lambda: 20,
    )
    result = profile.memory(SimpleNamespace(cuda=cuda))
    assert result == {
        "cuda_free_bytes": 200,
        "cuda_total_bytes": 1000,
        "torch_allocated_bytes": 10,
        "torch_reserved_bytes": 20,
        "system_available_bytes": 102400,
    }
    cuda.synchronize.assert_called_once()
