"""Compare speaker-embedding ONNX models on recordings from the robot.

Genuine: a recording of one person (first channel), split with Silero; scores are
leave-one-out against a profile built from the other segments. Impostors: either
digital clips (``digital:<dir>`` from build_impostor_set.py) or a recording of
the played-back impostor file (``play:<rec.wav>:<impostors_play.json> <offset_s>``).
Models: the default WeSpeaker ResNet34 + every .onnx in speaker_id/candidates/.

    python3 eval_models.py genuine.wav digital:/path/to/impostors
    python3 eval_models.py genuine.wav play:rec.wav:impostors_play.json 4.08
"""

import glob
import sys

import numpy as np
import soundfile as sf

sys.path.insert(0, "/workspace/src/hri/packages/speech")
from speech.speaker_id.embedder import SpeakerEmbedder, l2_normalize
from speech.speaker_id.segmenter import Segmenter, SegmenterConfig
from speech.speaker_id.silero import SileroVAD

A = "/workspace/src/hri/packages/speech/assets/"
D = A + "downloads/speaker_id/"


def segments(x, sr=16000, min_sec=1.0):
    vad = SileroVAD(A + "silero_vad.onnx")
    seg = Segmenter(SegmenterConfig())
    out, start = [], 0
    for i in range(0, len(x) - 511, 512):
        _, s, f = seg.push(x[i : i + 512], vad(x[i : i + 512]))
        if s:
            start = max(0, i - (len(s.audio) - 1) * 512)
        if f:
            out.append(x[start : i + 512])
    return [s for s in out if len(s) / sr >= min_sec]


MODELS = [D + "voxceleb_resnet34_LM.onnx"] + sorted(glob.glob(D + "candidates/*.onnx"))
genuine_wav = sys.argv[1]
imp_mode = sys.argv[2]  # "digital:<dir>" or "play:<rec wav>:<log json>"

g, _ = sf.read(genuine_wav, dtype="float32")
g = g[:, 0] if g.ndim > 1 else g
gsegs = segments(g)

imp = []  # (spk, wave)
if imp_mode.startswith("digital:"):
    for f in sorted(glob.glob(imp_mode.split(":", 1)[1] + "/digital_*.wav")):
        x, _ = sf.read(f, dtype="float32")
        spk = f.split("digital_")[1].rsplit("_", 1)[0]
        for k in range(0, max(1, len(x) // 48000)):  # 3 s chunks
            c = x[k * 48000 : (k + 1) * 48000]
            if len(c) >= 16000:
                imp.append((spk, c))
else:
    import json

    _, rec, logf = imp_mode.split(":")
    r, _ = sf.read(rec, dtype="float32")
    r = r[:, 0] if r.ndim > 1 else r
    log = json.load(open(logf))
    off = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    for e in log:
        a, b = int((e["start"] + off) * 16000), int((e["end"] + off) * 16000)
        imp.append((e["spk"], r[a:b]))
print(
    f"genuine segments {len(gsegs)}, impostor segments {len(imp)} from {len(set(s for s, _ in imp))} speakers"
)

for m in MODELS:
    emb = SpeakerEmbedder(m, ["cuda"])
    G = np.stack([emb.embed(s) for s in gsegs])
    loo = np.array(
        [l2_normalize(np.delete(G, i, 0).mean(0)) @ G[i] for i in range(len(G))]
    )
    anchor = l2_normalize(G.mean(0))
    imp_sc = np.array([anchor @ emb.embed(w) for _, w in imp])
    ths = np.linspace(-0.5, 1, 301)
    far = np.array([(imp_sc >= t).mean() for t in ths])
    frr = np.array([(loo < t).mean() for t in ths])
    k = np.argmin(np.abs(far - frr))
    t0 = ths[np.argmax(far == 0)] if (far == 0).any() else float("nan")
    print(
        f"{m.split('/')[-1]:32s} dim={emb.dim:4d} infer={emb.last_infer_ms:5.1f}ms | genuine LOO mean {loo.mean():.2f} min {loo.min():.2f} | impostor mean {imp_sc.mean():.2f} p95 {np.percentile(imp_sc, 95):.2f} max {imp_sc.max():.2f} | gap(min_gen-max_imp) {loo.min() - imp_sc.max():+.2f} | EER {100 * (far[k] + frr[k]) / 2:.1f}% @ {ths[k]:.2f} | FAR=0 from th {t0:.2f}"
    )
