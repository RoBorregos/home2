"""Kaldi-compatible log-mel filterbank features.

WeSpeaker models are trained on ``torchaudio.compliance.kaldi.fbank`` with
80 mel bins, 25 ms frames, 10 ms shift, hamming window and no dither, applied to
the waveform scaled to the int16 range, followed by per-utterance mean
normalisation (CMN). This module reproduces that in numpy so the robot does not
depend on torchaudio internals; ``use_torchaudio=True`` uses the reference
implementation when it is available.
"""

from __future__ import annotations

import numpy as np

_EPS = np.finfo(np.float32).eps


def _mel(freq: np.ndarray | float) -> np.ndarray | float:
    return 1127.0 * np.log(1.0 + np.asarray(freq) / 700.0)


def _mel_banks(
    num_bins: int, padded_window: int, sample_rate: int, low_freq: float = 20.0
) -> np.ndarray:
    """Triangular mel filters, same construction as Kaldi / torchaudio."""
    num_fft_bins = padded_window // 2
    nyquist = 0.5 * sample_rate
    fft_bin_width = sample_rate / padded_window
    mel_low = _mel(low_freq)
    mel_high = _mel(nyquist)
    mel_delta = (mel_high - mel_low) / (num_bins + 1)

    bins = np.arange(num_bins, dtype=np.float64)[:, None]
    left = mel_low + bins * mel_delta
    center = left + mel_delta
    right = center + mel_delta

    mel_freqs = _mel(fft_bin_width * np.arange(num_fft_bins, dtype=np.float64))[None, :]
    up = (mel_freqs - left) / (center - left)
    down = (right - mel_freqs) / (right - center)
    banks = np.maximum(0.0, np.minimum(up, down))
    # torchaudio pads one zero column for the Nyquist bin
    return np.pad(banks, ((0, 0), (0, 1))).astype(np.float32)


_BANK_CACHE: dict[tuple[int, int, int], np.ndarray] = {}


def kaldi_fbank(
    wave: np.ndarray,
    sample_rate: int = 16000,
    num_mel_bins: int = 80,
    frame_length_ms: float = 25.0,
    frame_shift_ms: float = 10.0,
) -> np.ndarray:
    """Log-mel filterbank of ``wave`` (float, already scaled to int16 range).

    Returns an array of shape ``[num_frames, num_mel_bins]`` (float32). Empty if
    the waveform is shorter than one frame.
    """
    wave = np.asarray(wave, dtype=np.float64)
    win = int(sample_rate * frame_length_ms / 1000)
    hop = int(sample_rate * frame_shift_ms / 1000)
    if wave.shape[0] < win:
        return np.zeros((0, num_mel_bins), dtype=np.float32)

    num_frames = 1 + (wave.shape[0] - win) // hop
    idx = np.arange(win)[None, :] + hop * np.arange(num_frames)[:, None]
    frames = wave[idx]

    frames = frames - frames.mean(axis=1, keepdims=True)  # remove DC offset
    prev = np.concatenate([frames[:, :1], frames[:, :-1]], axis=1)
    frames = frames - 0.97 * prev  # pre-emphasis
    frames = frames * np.hamming(win)[None, :]

    padded = 1 << (win - 1).bit_length()
    spectrum = np.abs(np.fft.rfft(frames, n=padded, axis=1)) ** 2

    key = (num_mel_bins, padded, sample_rate)
    if key not in _BANK_CACHE:
        _BANK_CACHE[key] = _mel_banks(num_mel_bins, padded, sample_rate)
    mel_energies = spectrum @ _BANK_CACHE[key].T.astype(np.float64)
    return np.log(np.maximum(mel_energies, _EPS)).astype(np.float32)


def speaker_features(
    wave_f32: np.ndarray, sample_rate: int = 16000, use_torchaudio: bool = False
) -> np.ndarray:
    """WeSpeaker input features for a float waveform in [-1, 1].

    Returns ``[num_frames, 80]`` with per-utterance mean removed.
    """
    scaled = np.asarray(wave_f32, dtype=np.float32) * 32768.0
    feats = None
    if use_torchaudio:
        try:
            import torch
            import torchaudio.compliance.kaldi as kaldi

            feats = (
                kaldi.fbank(
                    torch.from_numpy(scaled).unsqueeze(0),
                    num_mel_bins=80,
                    frame_length=25,
                    frame_shift=10,
                    dither=0.0,
                    sample_frequency=sample_rate,
                    window_type="hamming",
                    use_energy=False,
                )
                .numpy()
                .astype(np.float32)
            )
        except Exception:
            feats = None
    if feats is None:
        feats = kaldi_fbank(scaled, sample_rate=sample_rate)
    if feats.shape[0] == 0:
        return feats
    return feats - feats.mean(axis=0, keepdims=True)
