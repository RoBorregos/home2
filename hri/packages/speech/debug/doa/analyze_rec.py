"""Compare our SRP-PHAT DOA with the XVF-3000 chip DOA on a record_doa.py take.

    python3 analyze_rec.py <prefix> [win_sec] [mic_order e.g. 1234] [mirror 0/1]

Prints, per window: level over floor, our array-frame DOA, its confidence and
the chip's DOA (median of its VAD-on samples in that window).
"""

import json
import sys
import wave

import numpy as np
from speech.spatial.doa import ArrayGeometry, DoaConfig, SrpPhat, circ_diff

prefix = sys.argv[1]
win = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
order = [int(c) for c in (sys.argv[3] if len(sys.argv) > 3 else "1234")]
mirror = len(sys.argv) > 4 and sys.argv[4] == "1"

with wave.open(prefix + ".wav") as w:
    sr = w.getframerate()
    x = np.frombuffer(w.readframes(w.getnframes()), np.int16).reshape(-1, 6)
chip = json.load(open(prefix + ".json"))
mics = x[:, order].astype(np.float32) / 32768.0

cfg = DoaConfig(geometry=ArrayGeometry(mirror=mirror))
srp = SrpPhat(cfg, history_sec=win + 1)
step = int(win * sr)
rows = []
for s in range(0, mics.shape[0] - step + 1, step):
    srp.push(mics[s : s + step])
    est = srp.localize(step)
    t0, t1 = s / sr, (s + step) / sr
    cd = [c["doa"] for c in chip if t0 <= c.get("t", -1) < t1 and c.get("vad")]
    chip_doa = float(np.median(cd)) if cd else None
    lvl = 10 * np.log10(np.mean(mics[s : s + step] ** 2) + 1e-12)
    rows.append((t0, lvl, est, chip_doa))

print(
    f"noise floor (STFT band) {srp.noise_floor_db:.1f} dB | order {order} mirror {mirror}"
)
diffs = []
for t0, lvl, est, cd in rows:
    ours = (
        f"{est.array_deg:6.0f} conf {est.confidence:.2f} n{est.frames:3d}"
        if est
        else "     -"
    )
    srcs = " ".join(f"{s[0]:.0f}" for s in est.sources) if est else ""
    c = f"{cd:6.0f}" if cd is not None else "     -"
    d = ""
    if est and cd is not None and est.confidence > 0.3:
        dd = float(circ_diff(est.array_deg, cd))
        diffs.append(dd)
        d = f"diff {dd:5.0f}"
    print(f"{t0:5.1f}s {lvl:6.1f}dB  ours {ours}  [{srcs}]  chip {c}  {d}")
if diffs:
    print(
        f"\nours-chip: median {np.median(diffs):.0f}  "
        f"MAD {np.median(np.abs(np.array(diffs) - np.median(diffs))):.0f}  n={len(diffs)}"
    )
