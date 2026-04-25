#!/usr/bin/env python3
"""
AEC (Acoustic Echo Cancellation) Debug / Proof of Concept

Records audio from the ReSpeaker Mic Array v2.0 while simultaneously playing
a reference audio file. Applies echo cancellation using pyaec (SpeexDSP-based
AEC with built-in noise suppression preprocessing).

Also includes an echo gate (half-duplex) for comparison.

Requirements:
    pip install pyaec pyaudio pyusb

Usage:
    python3 aec_debug.py --play robot-voice.wav
    python3 aec_debug.py --play robot-voice.wav --duration 15
    python3 aec_debug.py --list-devices
"""

import argparse
import os
import threading
import time
import wave

import numpy as np
import pyaudio

# ── Constants ────────────────────────────────────────────────────────────────

SAMPLE_RATE = 16000
CHUNK_SIZE = 512
FORMAT = pyaudio.paInt16
RESPEAKER_CHANNELS = 6

RESPEAKER_VID = 0x2886
RESPEAKER_PID = 0x0018

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aec_output")

# pyaec parameters
AEC_FRAME_SIZE = 160  # 10ms at 16kHz (must match speex expectations)
AEC_FILTER_LENGTH = 3200  # 200ms tail at 16kHz (covers room echo)

# ── Check for pyaec ──────────────────────────────────────────────────────────

HAS_PYAEC = False
try:
    from pyaec import Aec

    HAS_PYAEC = True
except ImportError:
    pass


def respeaker_available() -> bool:
    try:
        import usb.core

        dev = usb.core.find(idVendor=RESPEAKER_VID, idProduct=RESPEAKER_PID)
        return dev is not None
    except (ImportError, Exception):
        return False


# ── Delay Estimation ─────────────────────────────────────────────────────────


def estimate_delay(
    mic: np.ndarray, ref: np.ndarray, max_delay_ms: float = 500.0
) -> int:
    """Estimate delay via GCC-PHAT cross-correlation."""
    max_lag = int(SAMPLE_RATE * max_delay_ms / 1000.0)
    n = min(len(mic), len(ref))

    chunk_len = min(n, SAMPLE_RATE * 4)
    start = max(0, (n - chunk_len) // 2)
    mic_c = mic[start : start + chunk_len] - np.mean(mic[start : start + chunk_len])
    ref_c = ref[start : start + chunk_len] - np.mean(ref[start : start + chunk_len])

    if np.std(mic_c) < 1.0 or np.std(ref_c) < 1.0:
        return 0

    fft_size = 1
    while fft_size < 2 * chunk_len:
        fft_size *= 2

    MIC = np.fft.rfft(mic_c, n=fft_size)
    REF = np.fft.rfft(ref_c, n=fft_size)
    cross = MIC * np.conj(REF)
    mag = np.abs(cross)
    mag[mag < 1e-10] = 1e-10
    xcorr = np.fft.irfft(cross / (mag**0.6), n=fft_size)

    pos = xcorr[:max_lag]
    neg = xcorr[-max_lag:]
    search = np.concatenate([pos, neg])
    indices = np.concatenate([np.arange(len(pos)), np.arange(-len(neg), 0)])

    best_idx = np.argmax(np.abs(search))
    delay = int(indices[best_idx])

    peak = np.abs(search[best_idx])
    mean_corr = np.mean(np.abs(search))
    snr = peak / (mean_corr + 1e-10)
    print(f"    Confidence: {snr:.1f}x (>3 = good)")
    if snr < 2.0:
        delay = 0

    return delay


def align_reference(ref: np.ndarray, delay: int, target_len: int) -> np.ndarray:
    aligned = np.zeros(target_len, dtype=np.float64)
    if delay >= 0:
        n = min(len(ref), target_len - delay)
        if n > 0:
            aligned[delay : delay + n] = ref[:n]
    else:
        off = -delay
        n = min(len(ref) - off, target_len)
        if n > 0 and off < len(ref):
            aligned[:n] = ref[off : off + n]
    return aligned


# ── pyaec AEC ────────────────────────────────────────────────────────────────


def pyaec_process(mic: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """
    Process audio through pyaec (SpeexDSP-based AEC).

    pyaec.Aec(frame_size, filter_length, sample_rate, enable_preprocess)
    - frame_size: samples per frame (160 = 10ms at 16kHz)
    - filter_length: echo tail length in samples (3200 = 200ms)
    - sample_rate: 16000
    - enable_preprocess: True enables built-in noise suppression

    cancel_echo(rec_buffer, echo_buffer) -> out_buffer
    - rec_buffer: microphone input (list of int16)
    - echo_buffer: speaker reference (list of int16)
    - returns: echo-cancelled audio (list of int16)
    """
    aec = Aec(AEC_FRAME_SIZE, AEC_FILTER_LENGTH, SAMPLE_RATE, True)

    n = min(len(mic), len(ref))
    mic_i16 = np.clip(mic[:n], -32768, 32767).astype(np.int16)
    ref_i16 = np.clip(ref[:n], -32768, 32767).astype(np.int16)

    output = np.zeros(n, dtype=np.int16)
    fs = AEC_FRAME_SIZE

    n_frames = n // fs
    for i in range(n_frames):
        s = i * fs
        e = s + fs

        rec_frame = mic_i16[s:e].tolist()
        echo_frame = ref_i16[s:e].tolist()

        out_frame = aec.cancel_echo(rec_frame, echo_frame)
        output[s:e] = np.array(out_frame, dtype=np.int16)

    return output.astype(np.float64)


# ── Echo Gate (half-duplex fallback) ─────────────────────────────────────────


def echo_gate(
    mic: np.ndarray,
    ref: np.ndarray,
    frame_ms: float = 30.0,
    fade_ms: float = 50.0,
    threshold_db: float = -40.0,
) -> np.ndarray:
    """Half-duplex: mute mic when speaker is active."""
    n = min(len(mic), len(ref))
    frame_samples = int(SAMPLE_RATE * frame_ms / 1000)
    fade_samples = int(SAMPLE_RATE * fade_ms / 1000)

    n_frames = (n + frame_samples - 1) // frame_samples
    ref_energy_db = np.full(n_frames, -100.0)

    for i in range(n_frames):
        s = i * frame_samples
        e = min(s + frame_samples, n)
        rms = np.sqrt(np.mean(ref[s:e] ** 2))
        if rms > 0:
            ref_energy_db[i] = 20 * np.log10(rms + 1e-10)

    gain = np.ones(n, dtype=np.float64)
    for i in range(n_frames):
        if ref_energy_db[i] > threshold_db:
            s = i * frame_samples
            e = min(s + frame_samples, n)
            gain[s:e] = 0.0

    if fade_samples > 0:
        alpha = 1.0 - np.exp(-2.2 / fade_samples)
        smoothed = np.zeros(n, dtype=np.float64)
        smoothed[0] = gain[0]
        for i in range(1, n):
            smoothed[i] = smoothed[i - 1] + alpha * (gain[i] - smoothed[i - 1])
        gain = smoothed

    return mic[:n] * gain


# ── Audio Utilities ──────────────────────────────────────────────────────────


def list_audio_devices():
    p = pyaudio.PyAudio()
    print("Available audio devices:")
    print(f"{'Index':>5}  {'Name':<40} {'In':>3} {'Out':>3} {'Rate':>6}")
    print("-" * 65)
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        print(
            f"{i:>5}  {info['name']:<40} {info['maxInputChannels']:>3} "
            f"{info['maxOutputChannels']:>3} {int(info['defaultSampleRate']):>6}"
        )
    p.terminate()


def save_wav(filepath: str, audio: np.ndarray, sample_rate: int = SAMPLE_RATE):
    audio_int16 = np.clip(audio, -32768, 32767).astype(np.int16)
    with wave.open(filepath, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(audio_int16.tobytes())
    print(f"  Saved: {filepath} ({len(audio_int16) / sample_rate:.1f}s)")


def load_reference_wav(filepath: str) -> np.ndarray:
    with wave.open(filepath, "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        rate = wf.getframerate()
        raw = wf.readframes(wf.getnframes())

    if sampwidth == 2:
        samples = np.frombuffer(raw, dtype=np.int16).astype(np.float64)
    elif sampwidth == 4:
        samples = np.frombuffer(raw, dtype=np.int32).astype(np.float64) / 65536.0
    else:
        raise ValueError(f"Unsupported sample width: {sampwidth}")

    if n_channels > 1:
        samples = samples.reshape(-1, n_channels)[:, 0]

    if rate != SAMPLE_RATE:
        original_len = len(samples)
        target_len = int(original_len * SAMPLE_RATE / rate)
        indices = np.linspace(0, original_len - 1, target_len)
        samples = np.interp(indices, np.arange(original_len), samples)
        print(f"    Resampled {rate}Hz → {SAMPLE_RATE}Hz")

    return samples


def play_audio_file_pyaudio(filepath: str, output_device_index=None):
    try:
        with wave.open(filepath, "rb") as wf:
            p = pyaudio.PyAudio()
            kwargs = {
                "format": p.get_format_from_width(wf.getsampwidth()),
                "channels": wf.getnchannels(),
                "rate": wf.getframerate(),
                "output": True,
            }
            if output_device_index is not None:
                kwargs["output_device_index"] = output_device_index
            stream = p.open(**kwargs)
            data = wf.readframes(1024)
            while data:
                stream.write(data)
                data = wf.readframes(1024)
            stream.stop_stream()
            stream.close()
            p.terminate()
    except Exception as e:
        print(f"[WARNING] Could not play audio: {e}")


# ── Main Pipeline ────────────────────────────────────────────────────────────


def record_with_aec(
    duration: float = 10.0,
    input_device_index=None,
    output_device_index=None,
    play_file: str = None,
):
    if not play_file or not os.path.exists(play_file):
        print("\n  [ERROR] --play <file.wav> is required.")
        return

    audio = pyaudio.PyAudio()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f"\n  Loading reference: {play_file}")
    ref_signal = load_reference_wav(play_file)
    print(f"    {len(ref_signal)} samples ({len(ref_signal)/SAMPLE_RATE:.1f}s)")

    print(f"\n  Recording {duration}s from ReSpeaker...")
    stream = audio.open(
        input_device_index=input_device_index,
        format=FORMAT,
        channels=RESPEAKER_CHANNELS,
        rate=SAMPLE_RATE,
        input=True,
        frames_per_buffer=CHUNK_SIZE,
    )

    play_started = threading.Event()

    def _play():
        play_started.set()
        play_audio_file_pyaudio(play_file, output_device_index)

    threading.Thread(target=_play, daemon=True).start()
    play_started.wait()
    print("  Playing + recording...")

    all_frames = []
    num_chunks = int(SAMPLE_RATE / CHUNK_SIZE * duration)
    progress_step = max(1, num_chunks // 10)
    for i in range(num_chunks):
        data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        all_frames.append(np.frombuffer(data, dtype=np.int16))
        if (i + 1) % progress_step == 0:
            print(f"  Recording: {(i+1)/num_chunks*100:.0f}%", end="\r")
    print("  Recording: 100% — Done.           ")

    stream.stop_stream()
    stream.close()
    audio.terminate()

    raw_interleaved = np.concatenate(all_frames)
    total_samples = len(raw_interleaved) // RESPEAKER_CHANNELS

    ch0_firmware = raw_interleaved[0::6].astype(np.float64)
    ch1_raw_mic = raw_interleaved[1::6].astype(np.float64)

    if len(ref_signal) < total_samples:
        ref_padded = np.zeros(total_samples, dtype=np.float64)
        ref_padded[: len(ref_signal)] = ref_signal
    else:
        ref_padded = ref_signal[:total_samples]

    print("\n  Channels:")
    print(f"    Ch0 (firmware AEC): RMS={np.sqrt(np.mean(ch0_firmware**2)):.1f}")
    print(f"    Ch1 (raw mic):      RMS={np.sqrt(np.mean(ch1_raw_mic**2)):.1f}")
    print(f"    Reference:          RMS={np.sqrt(np.mean(ref_padded**2)):.1f}")

    # ── Delay ─────────────────────────────────────────────────────────────
    print("\n  Estimating delay...")
    delay = estimate_delay(ch1_raw_mic, ref_padded)
    print(f"    Delay: {delay} samples ({delay/SAMPLE_RATE*1000:.1f} ms)")
    ref_aligned = align_reference(ref_padded, delay, total_samples)

    results = {}

    # ── pyaec (SpeexDSP AEC) ──────────────────────────────────────────────
    if HAS_PYAEC:
        print(
            f"\n  pyaec AEC (SpeexDSP, filter={AEC_FILTER_LENGTH} samples = "
            f"{AEC_FILTER_LENGTH/SAMPLE_RATE*1000:.0f}ms tail)..."
        )
        t0 = time.monotonic()
        cleaned_aec = pyaec_process(ch1_raw_mic, ref_aligned)
        elapsed = time.monotonic() - t0
        print(f"    Done in {elapsed:.2f}s (RTF={elapsed/duration:.3f})")
        results["pyaec"] = cleaned_aec
    else:
        print("\n  [WARNING] pyaec not installed. Install: pip install pyaec")

    # ── Echo Gate ─────────────────────────────────────────────────────────
    print("\n  Echo gate (half-duplex)...")
    t0 = time.monotonic()
    gated = echo_gate(ch1_raw_mic, ref_aligned)
    print(f"    Done in {time.monotonic()-t0:.2f}s")
    results["gate"] = gated

    # ── Metrics ──────────────────────────────────────────────────────────
    raw_rms = np.sqrt(np.mean(ch1_raw_mic**2))
    print("\n  Results:")
    print(f"    Raw mic:         RMS={raw_rms:.1f}")
    for name, data in results.items():
        rms = np.sqrt(np.mean(data**2))
        erle = 20 * np.log10(raw_rms / (rms + 1e-10))
        print(f"    {name:<14s}   RMS={rms:.1f}  ERLE={erle:.1f} dB")
    fw_rms = np.sqrt(np.mean(ch0_firmware**2))
    print(f"    firmware(ch0):   RMS={fw_rms:.1f}")

    # ── Save ─────────────────────────────────────────────────────────────
    print(f"\n  Saving to {OUTPUT_DIR}/")
    save_wav(os.path.join(OUTPUT_DIR, "raw_mic.wav"), ch1_raw_mic)
    save_wav(os.path.join(OUTPUT_DIR, "reference.wav"), ref_padded)
    save_wav(os.path.join(OUTPUT_DIR, "reference_aligned.wav"), ref_aligned)
    save_wav(os.path.join(OUTPUT_DIR, "firmware_aec.wav"), ch0_firmware)
    save_wav(os.path.join(OUTPUT_DIR, "aec_gate.wav"), gated)
    if "pyaec" in results:
        save_wav(os.path.join(OUTPUT_DIR, "aec_pyaec.wav"), results["pyaec"])

    all_6ch = raw_interleaved.reshape(-1, 6).astype(np.int16)
    sixch_path = os.path.join(OUTPUT_DIR, "all_6ch.wav")
    with wave.open(sixch_path, "wb") as wf:
        wf.setnchannels(6)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(all_6ch.tobytes())
    print(f"  Saved: {sixch_path} (6-channel)")

    print("\n" + "=" * 60)
    print("  Outputs:")
    if "pyaec" in results:
        print("    aec_pyaec.wav     — SpeexDSP AEC (echo cancellation)")
    print("    aec_gate.wav      — echo gate (half-duplex mute)")
    print("    firmware_aec.wav  — ReSpeaker firmware AEC (ch0)")
    print("    raw_mic.wav       — mic WITH echo (before)")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="AEC Debug: ReSpeaker + SpeexDSP echo cancellation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--play", type=str, required=True)
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args()

    if args.list_devices:
        list_audio_devices()
        return

    print("=" * 60)
    print(" AEC Debug — SpeexDSP AEC + Echo Gate")
    print("=" * 60)

    if not respeaker_available():
        print("  [ERROR] ReSpeaker not detected (USB 2886:0018).")
        return

    print("  ReSpeaker:  DETECTED")
    print(
        f"  pyaec:      {'AVAILABLE' if HAS_PYAEC else 'NOT INSTALLED (pip install pyaec)'}"
    )
    print(f"  Duration:   {args.duration}s")
    print(f"  Playback:   {args.play}")

    record_with_aec(
        duration=args.duration,
        input_device_index=args.input_device,
        output_device_index=args.output_device,
        play_file=args.play,
    )


if __name__ == "__main__":
    main()
