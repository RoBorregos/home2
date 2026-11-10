#!/usr/bin/env python3
"""Laptop side: stream this computer's mic (16 kHz mono int16) to remote_mic_receiver.py.

    ssh -N -L 5005:127.0.0.1:5005 orin@<orin-ip>      # in another terminal
    python remote_mic_sender.py [--device N]          # --list to see input devices
"""

import argparse
import socket

import pyaudio

CHUNK = 1024  # samples; same as audio_capturer.py

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5005)
ap.add_argument("--device", type=int, default=None)
ap.add_argument("--list", action="store_true")
args = ap.parse_args()

p = pyaudio.PyAudio()
if args.list:
    for i in range(p.get_device_count()):
        d = p.get_device_info_by_index(i)
        if d["maxInputChannels"] > 0:
            print(i, d["name"])
    raise SystemExit

stream = p.open(
    format=pyaudio.paInt16,
    channels=1,
    rate=16000,
    input=True,
    input_device_index=args.device,
    frames_per_buffer=CHUNK,
)
sock = socket.create_connection((args.host, args.port))
print("Streaming mic to the robot. Ctrl+C to stop.")
try:
    while True:
        sock.sendall(stream.read(CHUNK, exception_on_overflow=False))
except (KeyboardInterrupt, BrokenPipeError, ConnectionError):
    pass
finally:
    stream.close()
    p.terminate()
    sock.close()
