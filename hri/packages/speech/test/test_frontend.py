"""End-to-end SpatialFrontEnd on a synthetic scene (fake VAD, no models)."""

import numpy as np
from speech.spatial.doa import ArrayGeometry
from speech.spatial.frontend import FrontEndConfig, SpatialFrontEnd
from test_doa import SR, _plane_wave, _speechlike


class ScheduleVAD:
    """Returns 1.0 while the scene's talker is active (by sample count)."""

    def __init__(self, start):
        self.n = 0
        self.start = start

    def __call__(self, frame):
        self.n += frame.shape[0]
        return 1.0 if self.n > self.start else 0.0


def _scene(seconds=6, talk_from=2):
    geo = ArrayGeometry()
    rng = np.random.default_rng(3)
    noise = _plane_wave(
        rng.standard_normal(seconds * SR).astype(np.float32) * 0.05, 200, geo
    )
    speech = np.zeros_like(noise)
    speech[talk_from * SR :] = _plane_wave(
        _speechlike((seconds - talk_from) * SR, 2), 30, geo
    )
    return speech, noise


def _to_x6(mics):
    x6 = np.zeros((mics.shape[0], 6), dtype=np.int16)
    x6[:, 1:5] = np.clip(mics * 32768, -32768, 32767).astype(np.int16)
    return x6


def _run(fe, x6):
    out = [fe.process(x6[s : s + 1024]) for s in range(0, x6.shape[0] - 1023, 1024)]
    return np.concatenate(out)


def test_frontend_attenuates_directional_noise():
    speech, noise = _scene()
    fe = SpatialFrontEnd(ScheduleVAD(2 * SR), FrontEndConfig())
    _run(fe, _to_x6(speech + noise))  # learn noise (0-2 s) and talker (2-6 s)
    assert fe.bf.ready
    fe.bf.cfg.update_every = 10**9  # freeze the weights
    noise_bf = _run(fe, _to_x6(noise))[SR:]
    noise_mean = _run(
        SpatialFrontEnd(ScheduleVAD(2 * SR), FrontEndConfig(use_mvdr=False)),
        _to_x6(noise),
    )[SR:]
    # same talker gain (distortionless), noise from 200 deg at least 6 dB lower
    assert 20 * np.log10(np.std(noise_mean) / np.std(noise_bf)) > 6.0


def test_saying_forces_noise_label():
    fe = SpatialFrontEnd(lambda f: 1.0, FrontEndConfig())
    x6 = np.zeros((4096, 6), dtype=np.int16)
    fe.process(x6, saying=True)
    assert fe.speech is False
    fe.process(x6, saying=False)
    assert fe.speech is True
