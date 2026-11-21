"""Synthetic SNR-gain check for speech.spatial.beamformer.MvdrRtf."""

import copy

import numpy as np
from speech.spatial.beamformer import MvdrRtf
from speech.spatial.doa import ArrayGeometry
from test_doa import SR, _plane_wave, _speechlike


def _snr(s, n):
    return 10 * np.log10(np.sum(s**2) / np.sum(n**2))


def test_mvdr_gain_against_directional_noise():
    geo = ArrayGeometry()
    rng = np.random.default_rng(5)
    noise = _plane_wave(rng.standard_normal(6 * SR).astype(np.float32) * 0.05, 200, geo)
    noise += rng.standard_normal(noise.shape).astype(np.float32) * 0.002  # sensor noise
    speech = np.zeros_like(noise)
    speech[2 * SR :] = _plane_wave(_speechlike(4 * SR, 9), 30, geo)

    bf = MvdrRtf(4)
    bf.set_speech(False)
    bf.process((speech + noise)[: 2 * SR])
    bf.set_speech(True)
    bf.process((speech + noise)[2 * SR :])
    assert bf.ready

    frozen = copy.deepcopy(bf)
    frozen.cfg.update_every = 10**9
    s_out = copy.deepcopy(frozen).process(speech[2 * SR :])
    n_out = copy.deepcopy(frozen).process(noise[2 * SR :])
    gain = _snr(s_out[1024:], n_out[1024:]) - _snr(
        speech[2 * SR :, 0], noise[2 * SR :, 0]
    )
    assert gain > 6.0, gain


def test_passthrough_before_learning():
    bf = MvdrRtf(4)
    x = np.tile(_speechlike(SR, 1)[:, None], (1, 4))
    y = bf.process(x)
    # output is sample-aligned with the input; the first n_fft - hop samples
    # are only partially overlap-added
    head = bf.cfg.n_fft - bf.cfg.hop
    np.testing.assert_allclose(y[head:], x[head : y.shape[0], 0], atol=1e-3)
