"""Record the 6 ReSpeaker channels plus the on-chip DOA/VAD for offline DOA work.

ch0 = on-board processed (beamformed + NS + AGC), ch1-4 = raw mics,
ch5 = playback reference. The XVF-3000 DOAANGLE/VOICEACTIVITY registers are
polled every 100 ms in a side thread and stored next to the WAV as JSON.

Stop audio_capturer first: it holds the device.

    python3 record_doa.py <seconds> <out_prefix>
    -> <out_prefix>.wav (6 ch, 16 kHz, int16) + <out_prefix>.json
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
prefix = sys.argv[2] if len(sys.argv) > 2 else "/tmp/doa_rec"

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
tuning = Tuning(dev) if dev else None
chip = []
stop = threading.Event()
t0 = 0.0


def poll():
    while not stop.is_set():
        if tuning is not None:
            try:
                chip.append(
                    {
                        "t": round(time.time() - t0, 3),
                        "doa": int(tuning.direction),
                        "vad": int(tuning.is_voice()),
                        "speech": int(tuning.read("SPEECHDETECTED")),
                    }
                )
            except Exception as exc:  # USB hiccup: keep recording audio
                chip.append({"t": round(time.time() - t0, 3), "err": repr(exc)})
        time.sleep(0.1)


pa = pyaudio.PyAudio()
idx = next(
    i
    for i in range(pa.get_device_count())
    if "ReSpeaker" in pa.get_device_info_by_index(i)["name"]
    and pa.get_device_info_by_index(i)["maxInputChannels"] >= 6
)
st = pa.open(
    format=pyaudio.paInt16,
    channels=6,
    rate=16000,
    input=True,
    input_device_index=idx,
    frames_per_buffer=1024,
)
wait = float(sys.argv[3]) if len(sys.argv) > 3 else 0
for k in range(int(wait), 0, -1):
    print(f"  get ready... starting in {k:2d} s", end="\r", flush=True)
    time.sleep(1)
frames = []
t0 = time.time()
th = threading.Thread(target=poll, daemon=True)
th.start()
print(f"\n>>> RECORDING {secs:.0f}s -> {prefix}.wav  (speak now)\n", flush=True)
while time.time() - t0 < secs:
    frames.append(st.read(1024, exception_on_overflow=False))
    print(f"  {secs - (time.time() - t0):4.0f} s left", end="\r", flush=True)
stop.set()
th.join(timeout=1)
st.stop_stream()
st.close()
pa.terminate()
with wave.open(prefix + ".wav", "wb") as w:
    w.setnchannels(6)
    w.setsampwidth(2)
    w.setframerate(16000)
    w.writeframes(b"".join(frames))
with open(prefix + ".json", "w") as f:
    json.dump(chip, f)
doas = [c["doa"] for c in chip if c.get("vad")]
print(f"done; chip DOA samples with VAD: {len(doas)} {doas[:40]}", flush=True)
