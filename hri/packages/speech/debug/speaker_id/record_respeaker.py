"""Record all 6 ReSpeaker 4-mic channels to one WAV (ch0 = on-board processed,
ch1-4 = raw mics, ch5 = playback reference). Stop audio_capturer first: it holds
the device.

    python3 record_respeaker.py <seconds> <out.wav>
"""

import sys
import time
import wave

import pyaudio

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 40
out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/rec6.wav"
pa = pyaudio.PyAudio()
idx = next(
    i
    for i in range(pa.get_device_count())
    if "ReSpeaker" in pa.get_device_info_by_index(i)["name"]
    and pa.get_device_info_by_index(i)["maxInputChannels"] >= 6
)
name = pa.get_device_info_by_index(idx)["name"]
st = pa.open(
    format=pyaudio.paInt16,
    channels=6,
    rate=16000,
    input=True,
    input_device_index=idx,
    frames_per_buffer=1024,
)
frames = []
print(f"\n>>> GRABANDO {secs:.0f} s en {name}. HABLA AHORA.\n", flush=True)
t0 = time.time()
while time.time() - t0 < secs:
    frames.append(st.read(1024, exception_on_overflow=False))
    print(f"  {secs - (time.time() - t0):4.0f} s restantes", end="\r", flush=True)
st.stop_stream()
st.close()
pa.terminate()
with wave.open(out, "wb") as w:
    w.setnchannels(6)
    w.setsampwidth(2)
    w.setframerate(16000)
    w.writeframes(b"".join(frames))
print(f"\n>>> Listo, guardado {out}")
