#!/usr/bin/env python3
"""Small CLI to test audio_processing utilities on a WAV file.

Usage:
  python3 test_audio_utils.py input.wav output.wav

It will run reduce_noise -> regulate_gain and write `output.wav`.
"""

import sys
from pathlib import Path

from speech.audio_processing import load_wav, write_wav, reduce_noise, regulate_gain


def main():
    if len(sys.argv) < 3:
        print("Usage: test_audio_utils.py input.wav output.wav")
        return
    inp = Path(sys.argv[1])
    out = Path(sys.argv[2])
    if not inp.exists():
        print(f"Input not found: {inp}")
        return

    y, sr = load_wav(str(inp))
    print(f"Loaded {inp} sr={sr} len={len(y)}")

    y2 = reduce_noise(y, sr)
    y3 = regulate_gain(y2, target_rms=0.05)

    write_wav(str(out), y3, sr)
    print(f"Wrote processed file to {out}")


if __name__ == "__main__":
    main()
