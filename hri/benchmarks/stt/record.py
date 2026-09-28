import sys
from pathlib import Path

import sounddevice as sd
from scipy.io import wavfile

if len(sys.argv) > 1:
    target_path = Path(sys.argv[1])
    if target_path.suffix.lower() != ".wav":
        target_path = target_path.with_suffix(".wav")
    OUTPUT_FILE = str(target_path)
else:
    OUTPUT_FILE = "hello_frida.wav"

PULSE_SOURCE = "default"
SAMPLE_RATE = 16000
DURATION = 5

print(f"Recording {DURATION} seconds from PulseAudio (Channel 0)...")

recording = sd.rec(
    int(DURATION * SAMPLE_RATE),
    samplerate=SAMPLE_RATE,
    channels=1,
    device=PULSE_SOURCE,
    dtype="int16",
)
sd.wait()

wavfile.write(OUTPUT_FILE, SAMPLE_RATE, recording)

print(f"Successfully saved mono recording to: {OUTPUT_FILE}")
