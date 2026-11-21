"""StreamingDenoiser bookkeeping with a stand-in enhance function (no DFN)."""

import numpy as np
from speech.spatial.denoise import SR, StreamingConfig, StreamingDenoiser


def _tone_mix(n):
    t = np.arange(n) / SR
    return (
        0.3 * np.sin(2 * np.pi * 440 * t) + 0.1 * np.sin(2 * np.pi * 1234 * t)
    ).astype(np.float32)


def test_identity_is_seamless_with_odd_chunks():
    x = _tone_mix(3 * SR)
    den = StreamingDenoiser(lambda a: a, StreamingConfig())
    rng = np.random.default_rng(0)
    out, pos = [], 0
    while pos < x.shape[0]:
        n = int(rng.integers(100, 3000))
        out.append(den.process(x[pos : pos + n]))
        pos += n
    y = np.concatenate(out)
    assert y.shape[0] >= x.shape[0] - den.block - den.la
    # sample-aligned with the input, no seams (skip resampler start-up)
    err = np.abs(y[400:] - x[400 : y.shape[0]])
    assert err.max() < 0.02, err.max()


def test_gain_tracking_is_undone():
    """The normalisation must be inverted: a fn that only sees [-1, 1] input."""
    seen = []

    def fn(a):
        seen.append(np.abs(a).max())
        return a

    x = _tone_mix(2 * SR) * 0.01  # quiet mic
    den = StreamingDenoiser(fn)
    y = den.process(x)
    assert max(seen) > 0.5  # normalised before the model
    assert np.abs(y[400:]).max() < 0.02  # back at the input level


def test_attenuating_fn_attenuates():
    x = _tone_mix(2 * SR)
    den = StreamingDenoiser(lambda a: 0.5 * a)
    y = den.process(x)
    ratio = np.std(y[1000:]) / np.std(x[1000 : y.shape[0]])
    assert 0.45 < ratio < 0.55
