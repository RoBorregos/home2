import numpy as np
from scipy.signal import correlate
from typing import Tuple


class STFTEchoCanceller:
    """
    Short-Time Fourier Transform (STFT) echo canceller using a
    smoothed cross/auto PSD Wiener filter in each frequency bin.

    This produces both echo estimate (robot voice at microphone) and
    residual (microphone minus echo estimate).
    """

    def __init__(
        self,
        frame_size: int = 1024,
        hop_size: int = 256,
        beta: float = 0.9,
        sample_rate: int = 16000,
    ):
        """
        Initialize the STFT Echo Canceller.

        Args:
            frame_size: STFT frame size in samples
            hop_size: STFT hop size in samples
            beta: PSD smoothing factor [0..1], higher = more smoothing
            sample_rate: Audio sample rate in Hz
        """
        self.frame_size = frame_size
        self.hop_size = hop_size
        self.beta = beta
        self.sample_rate = sample_rate
        self.freq_bins = frame_size // 2 + 1
        self.win = np.hanning(frame_size)

        # Power Spectral Densities
        self.Pxx = np.ones(self.freq_bins) * 1e-10  # Auto-PSD of reference
        self.Pxd = np.ones(self.freq_bins, dtype=np.complex128) * 1e-10  # Cross-PSD

        # Buffers for streaming processing
        self.x_buffer = np.array([], dtype=np.float64)  # Reference buffer
        self.d_buffer = np.array([], dtype=np.float64)  # Microphone buffer
        self.output_buffer = np.array([], dtype=np.float64)  # Output buffer

        # Delay estimation
        self.estimated_delay = 0
        self.delay_locked = False

    def reset(self):
        """Reset the filter state."""
        self.Pxx[:] = 1e-10
        self.Pxd[:] = 1e-10
        self.x_buffer = np.array([], dtype=np.float64)
        self.d_buffer = np.array([], dtype=np.float64)
        self.output_buffer = np.array([], dtype=np.float64)
        self.estimated_delay = 0
        self.delay_locked = False

    def estimate_delay(
        self, ref: np.ndarray, mic: np.ndarray, max_lag_samps: int
    ) -> int:
        """
        Estimate echo delay (ref -> mic) via cross-correlation peak.
        Positive result means mic is delayed relative to ref (as expected).

        Args:
            ref: Reference signal (robot audio)
            mic: Microphone signal
            max_lag_samps: Maximum lag in samples to search

        Returns:
            Estimated delay in samples
        """
        # limit search window for speed
        n = min(len(ref), len(mic), self.sample_rate * 2)  # Max 2 seconds
        ref = ref[:n]
        mic = mic[:n]

        # Correlate mic with ref
        max_lag = max_lag_samps
        pad_ref = np.pad(ref, (max_lag, max_lag))
        c = correlate(mic, pad_ref, mode="valid")

        # Build lag axis: [-max_lag, ..., 0, ..., +max_lag]
        lags = np.arange(-max_lag, max_lag + 1)
        center = len(c) // 2
        window = c[center - max_lag : center + max_lag + 1]
        best = np.argmax(window)
        delay = lags[best]
        return int(delay)

    def _frame_iter(self, x: np.ndarray, d: np.ndarray):
        """
        Generator that yields frames from aligned signals.

        Args:
            x: Reference signal
            d: Microphone signal

        Yields:
            (frame_start_index, x_frame, d_frame)
        """
        N = self.frame_size
        H = self.hop_size
        for i in range(0, min(len(x), len(d)) - N + 1, H):
            xf = x[i : i + N] * self.win
            df = d[i : i + N] * self.win
            yield i, xf, df

    def process_block(
        self, x: np.ndarray, d: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process a block of audio (batch processing).

        Args:
            x: Reference (robot) signal
            d: Microphone (near-end + echo + noise) signal

        Returns:
            (echo_estimate, residual): Echo estimate and echo-cancelled signal
        """
        N = self.frame_size
        out_len = len(d)
        y_est = np.zeros(out_len)  # Echo estimate (robot at mic)
        e_out = np.zeros(out_len)  # Residual (echo-cancelled)

        for i, xf, df in self._frame_iter(x, d):
            X = np.fft.rfft(xf)
            D = np.fft.rfft(df)

            # Update PSDs (smoothed exponential averaging)
            self.Pxx = self.beta * self.Pxx + (1 - self.beta) * (np.abs(X) ** 2)
            self.Pxd = self.beta * self.Pxd + (1 - self.beta) * (np.conj(X) * D)

            # Wiener filter per bin: H = Pxd / (Pxx + eps)
            Hf = self.Pxd / (self.Pxx + 1e-12)

            # Estimate echo in frequency domain
            Y = Hf * X
            # Residual in frequency domain
            E = D - Y

            # Inverse STFT (Overlap-Add)
            y_frame = np.fft.irfft(Y, n=N) * self.win
            e_frame = np.fft.irfft(E, n=N) * self.win

            # Overlap-add
            y_est[i : i + N] += y_frame
            e_out[i : i + N] += e_frame

        return y_est, e_out

    def process_streaming(self, x_chunk: np.ndarray, d_chunk: np.ndarray) -> np.ndarray:
        """
        Process audio in streaming mode (real-time).

        Args:
            x_chunk: Reference (robot) audio chunk
            d_chunk: Microphone audio chunk

        Returns:
            Echo-cancelled audio chunk (may be shorter than input due to buffering)
        """
        # Add new data to buffers
        self.x_buffer = np.append(self.x_buffer, x_chunk)
        self.d_buffer = np.append(self.d_buffer, d_chunk)

        # Need at least one frame to process
        if len(self.x_buffer) < self.frame_size or len(self.d_buffer) < self.frame_size:
            return np.array([], dtype=np.float64)

        # Process available frames
        num_frames = (
            min(len(self.x_buffer), len(self.d_buffer)) - self.frame_size
        ) // self.hop_size

        if num_frames <= 0:
            return np.array([], dtype=np.float64)

        # Process frames
        process_length = num_frames * self.hop_size + self.frame_size
        x_process = self.x_buffer[:process_length]
        d_process = self.d_buffer[:process_length]

        # Run AEC
        _, e_out = self.process_block(x_process, d_process)

        # Keep only the latest output (to avoid latency buildup)
        output_length = num_frames * self.hop_size
        output = e_out[:output_length]

        # Remove processed samples from buffers
        self.x_buffer = self.x_buffer[output_length:]
        self.d_buffer = self.d_buffer[output_length:]

        return output

    def align_signals(
        self, ref: np.ndarray, mic: np.ndarray, max_delay_ms: float = 200.0
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        """
        Align reference and microphone signals by estimating and compensating for delay.

        Args:
            ref: Reference signal
            mic: Microphone signal
            max_delay_ms: Maximum delay to search in milliseconds

        Returns:
            (aligned_ref, aligned_mic, delay): Aligned signals and estimated delay
        """
        max_lag = int(max_delay_ms * 1e-3 * self.sample_rate)
        delay = self.estimate_delay(ref, mic, max_lag)

        # Align signals
        if delay > 0:
            # Mic is delayed, shift ref forward
            ref_aligned = np.pad(ref, (delay, 0))[: len(mic)]
            mic_aligned = mic[: len(ref_aligned)]
        elif delay < 0:
            # Ref is delayed, shift mic forward
            mic_aligned = np.pad(mic, (abs(delay), 0))[: len(ref)]
            ref_aligned = ref[: len(mic_aligned)]
        else:
            ref_aligned, mic_aligned = ref, mic

        # Ensure same length
        n = min(len(ref_aligned), len(mic_aligned))
        ref_aligned = ref_aligned[:n]
        mic_aligned = mic_aligned[:n]

        return ref_aligned, mic_aligned, delay


def normalize_audio(audio: np.ndarray, target_level: float = 0.8) -> np.ndarray:
    """
    Normalize audio to prevent clipping.

    Args:
        audio: Input audio signal
        target_level: Target peak level (0.0 to 1.0)

    Returns:
        Normalized audio
    """
    max_val = np.max(np.abs(audio))
    if max_val > 1e-6:
        return audio * (target_level / max_val) if max_val > target_level else audio
    return audio


def rms_normalize(audio: np.ndarray, target_rms: float = 0.1) -> np.ndarray:
    """
    Normalize audio by RMS level.

    Args:
        audio: Input audio signal
        target_rms: Target RMS level

    Returns:
        RMS-normalized audio
    """
    current_rms = np.sqrt(np.mean(audio**2) + 1e-12)
    if current_rms > 1e-6:
        return audio * (target_rms / current_rms)
    return audio
