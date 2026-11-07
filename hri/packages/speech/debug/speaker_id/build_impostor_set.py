"""Build an impostor set from public speech (LibriSpeech EN / MLS ES) clip URLs.

Input: JSON list of {"spk": ..., "url": ...} (e.g. audio "src" URLs from the
Hugging Face datasets-server /rows API). Writes digital_<spk>_<i>.wav for direct
scoring and impostors_play.wav + impostors_play.json (timings) to play from a
loudspeaker in front of the robot, so impostors go through the real room/mic.

    python3 build_impostor_set.py clips.json <out_dir>
"""

import io
import json
import os
import sys
import urllib.request

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

clips = json.load(open(sys.argv[1]))
outdir = sys.argv[2]
os.makedirs(outdir, exist_ok=True)
SR = 16000
by_spk = {}
for c in clips:
    by_spk.setdefault(c["spk"], []).append(c["url"])


def load(url):
    data = urllib.request.urlopen(url, timeout=60).read()
    x, sr = sf.read(io.BytesIO(data), dtype="float32")
    if x.ndim > 1:
        x = x.mean(axis=1)
    if sr != SR:
        x = resample_poly(x, SR, sr).astype("float32")
    return x


def densest(x, sec):
    n = int(sec * SR)
    if len(x) <= n:
        return x
    e = np.convolve(x**2, np.ones(SR // 10), "same")
    best = max(range(0, len(x) - n, SR // 4), key=lambda s: e[s : s + n].sum())
    return x[best : best + n]


digital = {}
for spk, urls in by_spk.items():
    xs = []
    for u in urls:
        try:
            xs.append(load(u))
        except Exception as exc:
            print("skip", spk, exc)
    digital[spk] = xs
    for i, x in enumerate(xs):
        sf.write(f"{outdir}/digital_{spk}_{i}.wav", x, SR)
print("digital speakers:", {k: len(v) for k, v in digital.items()})

# playback file: 6 en + 6 es, 3 clips of 3.5 s each, 1.5 s gaps, 2.5 s between speakers
play_spk = [s for s in digital if s.startswith("en")][:6] + [
    s for s in digital if s.startswith("es")
][:6]
parts, log, t = [np.zeros(int(2 * SR), "float32")], [], 2.0
for spk in play_spk:
    for x in digital[spk][:3]:
        y = densest(x, 3.5)
        y = 0.3 * y / (np.max(np.abs(y)) + 1e-9)
        log.append({"spk": spk, "start": round(t, 2), "end": round(t + len(y) / SR, 2)})
        parts += [y, np.zeros(int(1.5 * SR), "float32")]
        t += len(y) / SR + 1.5
    parts.append(np.zeros(int(1.0 * SR), "float32"))
    t += 1.0
play = np.concatenate(parts)
sf.write(f"{outdir}/impostors_play.wav", play, SR, subtype="PCM_16")
json.dump(log, open(f"{outdir}/impostors_play.json", "w"), indent=1)
print(f"playback file {len(play) / SR:.0f}s, speakers: {play_spk}")
