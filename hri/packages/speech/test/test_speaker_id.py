"""Unit tests for speech.speaker_id (no ROS needed).

python3 -m pytest hri/packages/speech/test/test_speaker_id.py -q
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from speech.speaker_id.embedder import l2_normalize  # noqa: E402
from speech.speaker_id.features import kaldi_fbank, speaker_features  # noqa: E402
from speech.speaker_id.profiles import ProfileStore, SpeakerProfile  # noqa: E402
from speech.speaker_id.segmenter import Segmenter, SegmenterConfig  # noqa: E402
from speech.speaker_id.verifier import (  # noqa: E402
    MIXED,
    NO_PROFILES,
    NOT_TARGET,
    TARGET,
    UNKNOWN,
    VerifierConfig,
    aggregate_segment,
    decide,
)

SR = 16000


def test_fbank_matches_torchaudio():
    torch = pytest.importorskip("torch")
    kaldi = pytest.importorskip("torchaudio.compliance.kaldi")
    rng = np.random.default_rng(0)
    t = np.arange(SR) / SR
    wave = (0.3 * np.sin(2 * np.pi * 220 * t) + 0.05 * rng.standard_normal(SR)) * 32768
    ref = kaldi.fbank(
        torch.from_numpy(wave.astype(np.float32)).unsqueeze(0),
        num_mel_bins=80,
        frame_length=25,
        frame_shift=10,
        dither=0.0,
        sample_frequency=SR,
        window_type="hamming",
        use_energy=False,
    ).numpy()
    ours = kaldi_fbank(wave.astype(np.float32))
    assert ours.shape == ref.shape
    assert np.max(np.abs(ours - ref)) < 1e-2


def test_features_shape_and_cmn():
    feats = speaker_features(np.random.default_rng(1).standard_normal(SR) * 0.1)
    assert feats.shape == (98, 80)
    assert np.allclose(feats.mean(axis=0), 0, atol=1e-4)


def _run_segmenter(probs, amps, cfg=None):
    cfg = cfg or SegmenterConfig(min_speech_sec=0.1, min_silence_sec=0.3, pad_sec=0.1)
    seg = Segmenter(cfg)
    rng = np.random.default_rng(2)
    started, finished = [], []
    for p, a in zip(probs, amps):
        frame = (a * rng.standard_normal(512)).astype(np.float32)
        _, s, f = seg.push(frame, p)
        if s:
            started.append(s)
        if f:
            finished.append(f)
    return seg, started, finished


def test_segmenter_opens_and_closes():
    probs = [0.05] * 60 + [0.95] * 40 + [0.05] * 30
    amps = [0.001] * 60 + [0.1] * 40 + [0.001] * 30
    _, started, finished = _run_segmenter(probs, amps)
    assert len(started) == 1 and len(finished) == 1
    dur = finished[0].duration(SR)
    assert 1.2 < dur < 1.8  # 40 frames (1.28 s) + pads


def test_segmenter_energy_gate_blocks_quiet_speech():
    # Silero says speech but the audio is at the noise floor -> no segment
    probs = [0.05] * 60 + [0.95] * 40
    amps = [0.01] * 60 + [0.01] * 40
    _, started, _ = _run_segmenter(probs, amps)
    assert started == []


def test_segmenter_mute_closes_segment():
    seg = Segmenter(SegmenterConfig(min_speech_sec=0.1))
    rng = np.random.default_rng(3)
    for _ in range(40):
        seg.push((0.001 * rng.standard_normal(512)).astype(np.float32), 0.0)
    for _ in range(20):
        seg.push((0.1 * rng.standard_normal(512)).astype(np.float32), 0.9)
    closed = seg.mute(True)
    assert closed is not None and closed.cut_by_tts
    info, s, _ = seg.push((0.1 * rng.standard_normal(512)).astype(np.float32), 0.9)
    assert info.muted and s is None


def _profile(name, vec):
    v = l2_normalize(np.asarray(vec, dtype=np.float32))
    return SpeakerProfile(name, v, v[None, :], "test", 6.0, 0.9)


def test_decide_labels():
    cfg = VerifierConfig()
    a = _profile("ana", [1, 0, 0])
    b = _profile("luis", [0, 1, 0])
    profiles = {"ana": a, "luis": b}
    near_a = l2_normalize(np.array([0.9, 0.1, 0.1], np.float32))
    assert decide(near_a, profiles, "ana", 2.0, cfg).label == TARGET
    assert decide(near_a, profiles, "luis", 2.0, cfg).label == NOT_TARGET
    assert decide(near_a, profiles, "ana", 0.5, cfg).label == UNKNOWN
    between = l2_normalize(np.array([0.7, 0.7, 0.1], np.float32))
    assert decide(between, profiles, "ana", 2.0, cfg).label == UNKNOWN
    assert decide(near_a, {}, "", 2.0, cfg).label == NO_PROFILES
    # empty target -> best enrolled profile is the target
    assert decide(near_a, profiles, "", 2.0, cfg).label == TARGET


def test_profile_store_roundtrip(tmp_path):
    store = ProfileStore(str(tmp_path), "test")
    prof = _profile("Mario", [0.2, 0.3, 0.9])
    prof.add_to_pool(np.array([0.1, 0.2, 0.9], np.float32), 10)
    store.save(prof)
    other = ProfileStore(str(tmp_path), "test")
    assert other.load_all() == []
    loaded = other.get("mario")
    assert loaded is not None and len(loaded.pool) == 1
    assert np.allclose(loaded.anchor, prof.anchor)
    # other backends are skipped
    assert ProfileStore(str(tmp_path), "other").load_all()
    assert other.forget("MARIO") and other.get("mario") is None


def test_aggregate_segment():
    T, N, U = TARGET, NOT_TARGET, UNKNOWN
    assert aggregate_segment(N, [T, T, N, N, U]) == MIXED
    assert aggregate_segment(U, [T, T, T, U]) == TARGET
    assert aggregate_segment(T, []) == TARGET
    assert aggregate_segment(N, [N, N, T]) == NOT_TARGET
    assert aggregate_segment(U, [U, T]) == UNKNOWN
