"""Synthetic checks for speech.spatial.doa (far-field plane waves, no ROS)."""

import numpy as np
import pytest
from speech.spatial.doa import (
    SPEED_OF_SOUND,
    ArrayGeometry,
    DoaConfig,
    SrpPhat,
    circ_diff,
    wrap180,
)

SR = 16000


def _plane_wave(sig, theta_deg, geo, sr=SR):
    """Delay a mono signal to each mic for a far-field source at theta (array frame)."""
    pos = geo.positions
    th = np.radians(theta_deg)
    u = np.array([np.cos(th), np.sin(th)])
    tau = -(pos @ u) / SPEED_OF_SOUND
    n = sig.shape[0]
    f = np.fft.rfftfreq(n, 1 / sr)
    S = np.fft.rfft(sig)
    return np.stack(
        [np.fft.irfft(S * np.exp(-2j * np.pi * f * t), n) for t in tau], axis=1
    )


def _speechlike(n, seed):
    rng = np.random.default_rng(seed)
    x = rng.standard_normal(n)
    env = 0.5 + 0.5 * np.sin(2 * np.pi * 3 * np.arange(n) / SR) ** 2
    return (x * env * 0.1).astype(np.float32)


@pytest.mark.parametrize("theta", [0, 37, 90, 145, 200, 271, 330])
def test_single_source(theta):
    geo = ArrayGeometry()
    rng = np.random.default_rng(1)
    noise = rng.standard_normal((SR, 4)).astype(np.float32) * 0.002
    sig = _plane_wave(_speechlike(SR, theta), theta, geo)
    srp = SrpPhat(DoaConfig(geometry=geo))
    srp.push(noise)  # silence model
    srp.push(sig + rng.standard_normal(sig.shape).astype(np.float32) * 0.002)
    est = srp.localize(SR)
    assert est is not None
    assert abs(circ_diff(est.array_deg, theta)) <= 6


def test_noise_model_rejects_directional_noise():
    """A steady fan at 90 deg must not win over a talker at 250 deg."""
    geo = ArrayGeometry()
    fan = _plane_wave(_speechlike(3 * SR, 7) * 0.6, 90, geo)
    talk = _plane_wave(_speechlike(SR, 8), 250, geo)
    srp = SrpPhat(DoaConfig(geometry=geo))
    srp.push(fan[: 2 * SR])
    srp.set_speech(True)
    srp.push(fan[2 * SR :] + talk)
    est = srp.localize(SR)
    assert abs(circ_diff(est.array_deg, 250)) <= 8


def test_two_sources():
    geo = ArrayGeometry()
    a = _plane_wave(_speechlike(SR, 3), 30, geo)
    b = _plane_wave(_speechlike(SR, 4), 200, geo)
    srp = SrpPhat(DoaConfig(geometry=geo))
    srp.push(
        np.random.default_rng(0).standard_normal((SR, 4)).astype(np.float32) * 1e-3
    )
    srp.push(a + b)
    est = srp.localize(SR)
    found = sorted(geo.to_array(s[0]) % 360 for s in est.sources)
    assert len(found) == 2
    assert abs(circ_diff(found[0], 30)) <= 15
    assert abs(circ_diff(found[1], 200)) <= 15


def test_robot_frame_mapping():
    geo = ArrayGeometry(offset_deg=-35.0, mirror=True)
    for a in (0, 45, 170, 300):
        assert abs(circ_diff(geo.to_array(geo.to_robot(a)), a)) < 1e-6
    assert wrap180(190) == -170
    assert wrap180(-180) == 180


def test_direction_label():
    from speech.spatial.doa import direction_label

    assert direction_label(0) == "front"
    assert direction_label(90) == "left"
    assert direction_label(-90) == "right"
    assert direction_label(-45) == "front-right"
    assert direction_label(179) == "back"
    assert direction_label(-130) == "back-right"
