#!/usr/bin/env python3
"""Record the 6 ReSpeaker channels (+ the chip's own DOA) for offline evaluation.

ch0 = chip-processed beam, ch1-4 = raw mics, ch5 = playback reference.
Stop audio_capturer first: it holds the device.

    python3 record.py <seconds> <out_prefix> [countdown_sec]
    -> <out_prefix>.wav (6 ch, 16 kHz, int16) + <out_prefix>.json (chip DOA/VAD)
"""

import json
import sys
import threading
import time
import wave

import pyaudio
import usb.core
from speech.tuning import Tuning

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10
prefix = sys.argv[2] if len(sys.argv) > 2 else "/tmp/speaker_id_rec"
countdown = float(sys.argv[3]) if len(sys.argv) > 3 else 0

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
tuning = Tuning(dev) if dev else None
chip, stop, t0 = [], threading.Event(), [0.0]


def poll_chip():
    while not stop.is_set():
        if tuning is not None:
            try:
                chip.append(
                    {
                        "t": round(time.time() - t0[0], 3),
                        "doa": int(tuning.direction),
                        "vad": int(tuning.is_voice()),
                    }
                )
            except usb.core.USBError as exc:  # USB hiccup: keep recording audio
                chip.append({"t": round(time.time() - t0[0], 3), "err": repr(exc)})
        time.sleep(0.1)


pa = pyaudio.PyAudio()
idx = next(
    i
    for i in range(pa.get_device_count())
    if "ReSpeaker" in pa.get_device_info_by_index(i)["name"]
    and pa.get_device_info_by_index(i)["maxInputChannels"] >= 6
)
stream = pa.open(
    format=pyaudio.paInt16,
    channels=6,
    rate=16000,
    input=True,
    input_device_index=idx,
    frames_per_buffer=1024,
)
for k in range(int(countdown), 0, -1):
    print(f"  get ready... starting in {k:2d} s", end="\r", flush=True)
    time.sleep(1)

frames = []
t0[0] = time.time()
threading.Thread(target=poll_chip, daemon=True).start()
print(f"\n>>> RECORDING {secs:.0f} s -> {prefix}.wav (speak now)\n", flush=True)
while time.time() - t0[0] < secs:
    frames.append(stream.read(1024, exception_on_overflow=False))
    print(f"  {secs - (time.time() - t0[0]):4.0f} s left", end="\r", flush=True)
stop.set()
stream.stop_stream()
stream.close()
pa.terminate()

with wave.open(prefix + ".wav", "wb") as w:
    w.setnchannels(6)
    w.setsampwidth(2)
    w.setframerate(16000)
    w.writeframes(b"".join(frames))
with open(prefix + ".json", "w") as f:
    json.dump(chip, f)
print(f"\nsaved {prefix}.wav and {prefix}.json")
