"""Synthetic checks for speaker id + direction (no ROS, no models, no mic).

python3 -m pytest hri/benchmarks/speaker_id/test_speaker_id.py -q
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages" / "speech"))

from speech.spatial_utils import (  # noqa: E402
    SPEED_OF_SOUND,
    ArrayGeometry,
    DoaConfig,
    MvdrRtf,
    SpeakerTracker,
    SrpPhat,
    circ_diff,
    direction_label,
    merge_doas,
)
from speech.speaker_id_utils import (  # noqa: E402
    MIXED,
    NO_PROFILES,
    NOT_TARGET,
    TARGET,
    UNKNOWN,
    Segmenter,
    SegmenterConfig,
    SpeakerProfile,
    VerifierConfig,
    aggregate_segment,
    decide,
    l2_normalize,
    speaker_features,
)

SR = 16000


def _plane_wave(sig, theta_deg, geo):
    """Far-field source at theta (array frame) as heard by each of the 4 mics."""
    th = np.radians(theta_deg)
    tau = -(geo.positions @ np.array([np.cos(th), np.sin(th)])) / SPEED_OF_SOUND
    f = np.fft.rfftfreq(sig.shape[0], 1 / SR)
    S = np.fft.rfft(sig)
    return np.stack(
        [np.fft.irfft(S * np.exp(-2j * np.pi * f * t), sig.shape[0]) for t in tau],
        axis=1,
    ).astype(np.float32)


def _speechlike(n, seed):
    x = np.random.default_rng(seed).standard_normal(n)
    env = 0.5 + 0.5 * np.sin(2 * np.pi * 3 * np.arange(n) / SR) ** 2
    return (x * env * 0.1).astype(np.float32)


# ------------------------------------------------------------- direction
@pytest.mark.parametrize("theta", [0, 90, 145, 271])
def test_doa_single_source(theta):
    geo = ArrayGeometry()
    rng = np.random.default_rng(1)
    srp = SrpPhat(DoaConfig(geometry=geo))
    srp.push(rng.standard_normal((SR, 4)).astype(np.float32) * 0.002)  # silence
    srp.push(_plane_wave(_speechlike(SR, theta), theta, geo))
    assert abs(circ_diff(srp.localize(SR).array_deg, theta)) <= 6


def test_doa_ignores_learnt_directional_noise():
    """A steady noise source at 90 deg must not win over a talker at 250 deg."""
    geo = ArrayGeometry()
    fan = _plane_wave(_speechlike(3 * SR, 7) * 0.6, 90, geo)
    srp = SrpPhat(DoaConfig(geometry=geo))
    srp.push(fan[: 2 * SR])
    srp.set_speech(True)
    srp.push(fan[2 * SR :] + _plane_wave(_speechlike(SR, 8), 250, geo))
    assert abs(circ_diff(srp.localize(SR).array_deg, 250)) <= 8


def test_robot_frame_and_labels():
    geo = ArrayGeometry(offset_deg=-15.0)
    assert abs(float(geo.to_robot(15.0))) < 1e-6  # calibrated front
    assert direction_label(0) == "front" and direction_label(-90) == "right"
    assert direction_label(90) == "left" and direction_label(180) == "back"


def test_merge_doas_uses_reliable_windows_and_wraps():
    d = merge_doas(
        [
            {"azimuth_deg": -95.0, "confidence": 0.6},
            {"azimuth_deg": -105.0, "confidence": 0.6},
            {"azimuth_deg": 40.0, "confidence": 0.05},  # unreliable: ignored
        ],
        0.15,
    )
    assert abs(d["azimuth_deg"] + 100.0) < 1.0 and d["direction"] == "right"
    d = merge_doas(
        [
            {"azimuth_deg": 175.0, "confidence": 0.5},
            {"azimuth_deg": -175.0, "confidence": 0.5},
        ],
        0.15,
    )
    assert abs(abs(d["azimuth_deg"]) - 180.0) < 1.0
    assert merge_doas([None], 0.15) is None


# --------------------------------------------------------------- tracker
def test_tracker_attributes_unsure_voice_by_direction():
    tr = SpeakerTracker()
    assert tr.update(0.0, "mario", True, -95.0) == ("mario", "voice")
    assert tr.update(2.0, None, False, -100.0) == ("mario", "direction")
    assert tr.update(2.5, None, False, 40.0) == ("unknown", "none")
    assert tr.update(30.0, None, False, -100.0) == ("unknown", "none")  # expired


def test_tracker_never_names_a_rejected_voice():
    tr = SpeakerTracker()
    tr.update(0.0, "mario", True, -95.0)
    assert tr.update(1.0, None, False, -95.0, rejected=True) == ("unknown", "none")
    assert tr.update(1.0, None, False, -95.0, doa_reliable=False) == (
        "unknown",
        "none",
    )


# ------------------------------------------------------------ beamformer
def test_mvdr_attenuates_directional_noise():
    geo = ArrayGeometry()
    rng = np.random.default_rng(5)
    noise = _plane_wave(rng.standard_normal(6 * SR).astype(np.float32) * 0.05, 200, geo)
    noise += rng.standard_normal(noise.shape).astype(np.float32) * 0.002
    speech = np.zeros_like(noise)
    speech[2 * SR :] = _plane_wave(_speechlike(4 * SR, 9), 30, geo)
    bf = MvdrRtf(4)
    bf.process((speech + noise)[: 2 * SR])  # noise only
    bf.set_speech(True)
    bf.process((speech + noise)[2 * SR :])
    assert bf.ready
    bf.cfg.update_every = 10**9  # freeze the weights
    w = bf.w.copy()

    def out(x):
        bf.w, bf._in, bf._ola = w, np.zeros((0, 4), np.float32), bf._ola * 0
        return bf.process(x)

    def snr(s, n):
        return 10 * np.log10(np.sum(s**2) / np.sum(n**2))

    gain = snr(out(speech[2 * SR :])[1024:], out(noise[2 * SR :])[1024:]) - snr(
        speech[2 * SR :, 0], noise[2 * SR :, 0]
    )
    assert gain > 6.0, gain


# ------------------------------------------------------------ speaker id
def test_features_shape_and_cmn():
    feats = speaker_features(np.random.default_rng(1).standard_normal(SR) * 0.1)
    assert feats.shape == (98, 80)
    assert np.allclose(feats.mean(axis=0), 0, atol=1e-4)


def test_segmenter_opens_closes_and_gates_quiet_audio():
    def run(probs, amps):
        seg = Segmenter(
            SegmenterConfig(min_speech_sec=0.1, min_silence_sec=0.3, pad_sec=0.1)
        )
        rng = np.random.default_rng(2)
        events = []
        for p, a in zip(probs, amps):
            _, s, f = seg.push((a * rng.standard_normal(512)).astype(np.float32), p)
            events += [("start", s)] * bool(s) + [("end", f)] * bool(f)
        return events

    ev = run(
        [0.05] * 60 + [0.95] * 40 + [0.05] * 30,
        [0.001] * 60 + [0.1] * 40 + [0.001] * 30,
    )
    assert [e for e, _ in ev] == ["start", "end"]
    assert 1.2 < ev[1][1].duration(SR) < 1.8
    # Silero says speech but the audio is at the noise floor -> no segment
    assert run([0.05] * 60 + [0.95] * 40, [0.01] * 100) == []


def _profile(name, vec):
    v = l2_normalize(np.asarray(vec, dtype=np.float32))
    return SpeakerProfile(name, v, v[None, :], "test", 6.0, 0.9)


def test_decide_and_aggregate():
    cfg = VerifierConfig()
    profiles = {"ana": _profile("ana", [1, 0, 0]), "luis": _profile("luis", [0, 1, 0])}
    near_ana = l2_normalize(np.array([0.9, 0.1, 0.1], np.float32))
    between = l2_normalize(np.array([0.7, 0.7, 0.1], np.float32))
    assert decide(near_ana, profiles, "ana", 2.0, cfg).label == TARGET
    assert decide(near_ana, profiles, "luis", 2.0, cfg).label == NOT_TARGET
    assert decide(near_ana, profiles, "ana", 0.5, cfg).label == UNKNOWN  # too short
    assert decide(between, profiles, "ana", 2.0, cfg).label == UNKNOWN
    assert decide(near_ana, {}, "", 2.0, cfg).label == NO_PROFILES
    T, N, U = TARGET, NOT_TARGET, UNKNOWN
    assert aggregate_segment(N, [T, T, N, N, U]) == MIXED
    assert aggregate_segment(U, [T, T, T, U]) == TARGET
    assert aggregate_segment(N, [N, N, T]) == NOT_TARGET
