"""Audio processing utilities used by the speech package.

Provides: reduce_noise (spectral gating), regulate_gain, compress_dynamic_range,
adaptive_agc, dereverb_spectral, generate_comfort_noise, separate_sources,
and small helpers for loading/writing audio.

The implementations are conservative and depend only on numpy/scipy/soundfile.
"""
from typing import Optional, Tuple
import numpy as np
import scipy.signal
import soundfile as sf

try:
    from sklearn.decomposition import FastICA
    _HAS_SKLEARN = True
except Exception:
    _HAS_SKLEARN = False


def load_wav(path: str, target_sr: Optional[int] = None) -> Tuple[np.ndarray, int]:
    data, sr = sf.read(path)
    if target_sr is not None and sr != target_sr:
        # simple resample
        number_of_samples = round(len(data) * float(target_sr) / sr)
        data = scipy.signal.resample(data, number_of_samples)
        sr = target_sr
    # ensure mono
    if data.ndim > 1:
        data = np.mean(data, axis=1)
    return data.astype(np.float32), sr


def write_wav(path: str, y: np.ndarray, sr: int):
    # clip to [-1,1]
    y = np.asarray(y)
    if y.dtype.kind == "f":
        y = np.clip(y, -1.0, 1.0)
    sf.write(path, y, sr)


def _rms(y: np.ndarray) -> float:
    return np.sqrt(np.mean(y ** 2) + 1e-12)


def regulate_gain(y: np.ndarray, target_rms: float = 0.05) -> np.ndarray:
    """Scale audio to reach target RMS level (linear)."""
    y = y.astype(np.float32)
    cur = _rms(y)
    if cur < 1e-9:
        return y
    gain = float(target_rms / cur)
    return y * gain


def compress_dynamic_range(y: np.ndarray, threshold_db: float = -20.0, ratio: float = 2.0) -> np.ndarray:
    """Simple per-sample soft compressor. Conservative and fast.

    threshold_db: level in dBFS where compression begins (negative value)
    ratio: compression ratio (>1)
    """
    eps = 1e-12
    y = y.astype(np.float32)
    mag = np.abs(y) + eps
    mag_db = 20.0 * np.log10(mag)
    over = mag_db > threshold_db
    gain_db = np.zeros_like(mag_db)
    gain_db[over] = (threshold_db + (mag_db[over] - threshold_db) / ratio) - mag_db[over]
    gain = 10.0 ** (gain_db / 20.0)
    return np.sign(y) * mag * gain


def reduce_noise(
    y: np.ndarray,
    sr: int,
    noise_clip: Optional[np.ndarray] = None,
    n_fft: int = 2048,
    hop_length: int = 512,
    n_std_thresh: float = 1.5,
    prop_decrease: float = 1.0,
) -> np.ndarray:
    """Spectral gating noise reduction. Estimates noise from noise_clip or first 0.5s.

    Returns processed signal in same shape as input.
    """
    y = y.astype(np.float32)
    if noise_clip is None:
        # use first 0.5s or less
        n_noise = min(len(y), int(0.5 * sr))
        if n_noise <= 0:
            return y
        noise_clip = y[:n_noise]

    # STFT
    f, t, S = scipy.signal.stft(y, fs=sr, nperseg=n_fft, noverlap=n_fft - hop_length)
    _, _, N = scipy.signal.stft(noise_clip, fs=sr, nperseg=n_fft, noverlap=n_fft - hop_length)

    S_mag = np.abs(S)
    N_mag = np.abs(N)

    # Estimate noise mean and std per-bin
    noise_mean = np.mean(N_mag, axis=1, keepdims=True)
    noise_std = np.std(N_mag, axis=1, keepdims=True)

    # Threshold
    thresh = noise_mean + n_std_thresh * noise_std
    mask_gain = 1.0 - prop_decrease * np.minimum(1.0, np.maximum(0.0, (thresh - S_mag) / (S_mag + 1e-12)))

    S_filtered = S * mask_gain
    # Inverse STFT
    _, y_out = scipy.signal.istft(S_filtered, fs=sr, nperseg=n_fft, noverlap=n_fft - hop_length)
    # match length
    if len(y_out) > len(y):
        y_out = y_out[: len(y)]
    elif len(y_out) < len(y):
        y_out = np.pad(y_out, (0, len(y) - len(y_out)))
    return y_out.astype(np.float32)


def adaptive_agc(y: np.ndarray, target_rms: float = 0.05, frame_size: int = 1024, hop: int = 512) -> Tuple[np.ndarray, float]:
    """Frame-based AGC: smooth per-frame gains to bring RMS to target.

    Returns (y_out, final_gain_mean)
    """
    y = y.astype(np.float32)
    out = np.zeros_like(y)
    gains = []
    alpha = 0.9
    prev_gain = 1.0
    for start in range(0, len(y), hop):
        frame = y[start : start + frame_size]
        if len(frame) == 0:
            break
        cur = _rms(frame)
        if cur < 1e-9:
            g = prev_gain
        else:
            g = target_rms / cur
        # smooth
        g = alpha * prev_gain + (1 - alpha) * g
        gains.append(g)
        out[start : start + len(frame)] += frame * g
        prev_gain = g

    final_gain = float(np.mean(gains)) if gains else 1.0
    return out, final_gain


def dereverb_spectral(y: np.ndarray, sr: int, decay_scale: float = 0.8, n_fft: int = 2048, hop_length: int = 512) -> np.ndarray:
    """Light dereverberation by spectral median subtraction across time."""
    f, t, S = scipy.signal.stft(y, fs=sr, nperseg=n_fft, noverlap=n_fft - hop_length)
    mag = np.abs(S)
    median_spec = np.median(mag, axis=1, keepdims=True)
    reduced = np.maximum(0.0, mag - decay_scale * median_spec)
    phase = np.angle(S)
    S_new = reduced * np.exp(1j * phase)
    _, y_out = scipy.signal.istft(S_new, fs=sr, nperseg=n_fft, noverlap=n_fft - hop_length)
    if len(y_out) > len(y):
        y_out = y_out[: len(y)]
    elif len(y_out) < len(y):
        y_out = np.pad(y_out, (0, len(y) - len(y_out)))
    return y_out.astype(np.float32)


def generate_comfort_noise(noise_profile: np.ndarray, length: int, sr: int, level_db: float = -50.0) -> np.ndarray:
    """Generate comfort noise matching the provided magnitude profile.

    `noise_profile` should be a frequency magnitude vector (linear) or a time-frequency matrix.
    For simplicity, if provided a 1D vector, we create noise in freq domain and ISTFT.
    """
    # If 1D, use as magnitude for one frame and tile
    if noise_profile.ndim == 1:
        mag = np.tile(noise_profile[:, None], (1, int(np.ceil(length / 1024))))
    else:
        mag = noise_profile

    n_fft = (mag.shape[0] - 1) * 2
    hop = 256
    frames = mag.shape[1]
    # Randomize phase
    phase = np.exp(1j * 2 * np.pi * np.random.rand(*mag.shape))
    S = mag * phase
    _, y = scipy.signal.istft(S, fs=sr, nperseg=n_fft, noverlap=n_fft - hop)
    y = y[:length]
    # scale by level_db
    rms = _rms(y)
    if rms > 1e-12:
        target = 10 ** (level_db / 20.0)
        y = y * (target / rms)
    return y.astype(np.float32)


def separate_sources(y: np.ndarray, sr: int, n_sources: int = 2) -> np.ndarray:
    """Primitive single-channel separation using FastICA on magnitude spectrogram.

    Returns array shape (n_sources, len(y)). Requires scikit-learn.
    """
    if not _HAS_SKLEARN:
        raise RuntimeError("separate_sources requires scikit-learn (FastICA) to be installed")

    S = np.abs(scipy.signal.stft(y, fs=sr, nperseg=1024, noverlap=768)[2])
    # shape (freq, time) -> transpose to (time, freq)
    X = S.T
    ica = FastICA(n_components=min(n_sources, X.shape[1]), max_iter=500)
    try:
        S_ = ica.fit_transform(X)
    except Exception as e:
        raise
    # Convert back: create simple masks per component and reconstruct
    components = S_.T
    sources = []
    for comp in components:
        mask = np.abs(comp)[:, None]
        # apply mask to original STFT magnitude preserving phase
        # reconstruct by multiplying magnitude mask
        mag_mask = (mask.T / (np.sum(np.abs(components), axis=0) + 1e-12))[0]
        # broadcast to freq bins
        mag_mask = np.tile(mag_mask[:, None], (1, S.shape[0])).T
        # get STFT of original to combine with mask
        f, t, ST = scipy.signal.stft(y, fs=sr, nperseg=1024, noverlap=768)
        S_comp = np.abs(ST) * mag_mask
        _, y_comp = scipy.signal.istft(S_comp * np.exp(1j * np.angle(ST)), fs=sr, nperseg=1024, noverlap=768)
        y_comp = y_comp[: len(y)]
        sources.append(y_comp)

    return np.vstack(sources)
