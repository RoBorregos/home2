#!/usr/bin/env python3
"""Offline evaluation on 6-channel recordings from record.py.

Speaker id: genuine = recording of the enrolled person, impostor = recording of
other people. Genuine segments are scored leave-one-out against a profile built
from the rest; impostor segments against the full profile.

    python3 evaluate.py speaker genuine.wav impostor.wav            # ch0 vs MVDR
    python3 evaluate.py speaker genuine.wav impostor.wav --models   # + candidate models

Direction (calibration of DOA_OFFSET_DEG): talk from a known position and check
the azimuth in the robot frame (0 = front, +90 = left, -90 = right).

    python3 evaluate.py doa front.wav --offset -15
"""

import argparse
import glob
import sys
from pathlib import Path

import numpy as np
import soundfile as sf

SPEECH = Path(__file__).resolve().parents[2] / "packages" / "speech"
sys.path.insert(0, str(SPEECH))

from speech.spatial_utils import (  # noqa: E402
    ArrayGeometry,
    DoaConfig,
    FrontEndConfig,
    MvdrFrontEnd,
    direction_label,
    srp_offline,
)
from speech.speaker_id_utils import (  # noqa: E402
    FRAME_SAMPLES,
    SileroVAD,
    SpeakerEmbedder,
    l2_normalize,
)

SR = 16000
CHUNK = 1024
SILERO = str(SPEECH / "assets" / "silero_vad.onnx")
MODELS = SPEECH / "assets" / "downloads" / "speaker_id"


def load6(path):
    x, sr = sf.read(path, dtype="int16")
    assert sr == SR and x.ndim == 2 and x.shape[1] == 6, f"{path}: need 6 ch @ 16 kHz"
    return x


def mvdr(x6):
    fe = MvdrFrontEnd(SileroVAD(SILERO), FrontEndConfig(output_gain_db=10.0))
    y = np.concatenate(
        [fe.process(x6[s : s + CHUNK]) for s in range(0, len(x6) - CHUNK + 1, CHUNK)]
    )
    return np.pad(y, (0, len(x6) - len(y)))


def speech_segments(x6, min_sec=1.0, max_sec=3.0):
    """1-3 s speech chunks (Silero on the raw mic average)."""
    vad = SileroVAD(SILERO)
    mono = np.clip(x6[:, 1:5].astype(np.float32).mean(1) / 32768 * 4, -1, 1)
    mask = np.zeros(len(mono), bool)
    for s in range(0, len(mono) - FRAME_SAMPLES + 1, FRAME_SAMPLES):
        mask[s : s + FRAME_SAMPLES] = vad(mono[s : s + FRAME_SAMPLES]) >= 0.5
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 0.4 * SR)
    runs = zip(np.r_[idx[0], idx[breaks + 1]], np.r_[idx[breaks], idx[-1]])
    out = []
    for a, b in runs:
        for s in range(a, b, int(max_sec * SR)):
            e = min(b, s + int(max_sec * SR))
            if e - s >= min_sec * SR:
                out.append((s, e))
    return out


def separation(emb, gen, imp):
    G = np.stack([emb.embed(w) for w in gen])
    loo = np.array(
        [l2_normalize(np.delete(G, i, 0).mean(0)) @ G[i] for i in range(len(G))]
    )
    imp_s = np.array([l2_normalize(G.mean(0)) @ emb.embed(w) for w in imp])
    d = (loo.mean() - imp_s.mean()) / np.sqrt(0.5 * (loo.var() + imp_s.var()) + 1e-12)
    return loo, imp_s, d


def cmd_speaker(args):
    g6, i6 = load6(args.genuine), load6(args.impostor)
    gseg, iseg = speech_segments(g6), speech_segments(i6)
    print(f"{len(gseg)} genuine / {len(iseg)} impostor segments")
    sources = {
        "ch0 (chip)": (g6[:, 0] / 32768.0, i6[:, 0] / 32768.0),
        "mvdr": (mvdr(g6), mvdr(i6)),
    }
    models = [MODELS / "voxceleb_resnet34_LM.onnx"]
    if args.models:
        models += [
            Path(p) for p in sorted(glob.glob(str(MODELS / "candidates" / "*.onnx")))
        ]
    print(
        f"{'model':<30}{'input':<12}{'gen_min':>8}{'gen_med':>8}{'imp_max':>8}{'d_prime':>8}{'ms':>6}"
    )
    for model in models:
        emb = SpeakerEmbedder(str(model), ["cuda", "cpu"], SR)
        emb.warmup()
        for name, (g, i) in sources.items():
            if args.models and model != models[0] and name != "mvdr":
                continue  # candidates only on the production input
            gen = [g[a:b].astype(np.float32) for a, b in gseg]
            imp = [i[a:b].astype(np.float32) for a, b in iseg]
            loo, imp_s, d = separation(emb, gen, imp)
            print(
                f"{model.stem:<30}{name:<12}{loo.min():8.2f}{np.median(loo):8.2f}"
                f"{imp_s.max():8.2f}{d:8.2f}{emb.last_infer_ms:6.0f}"
            )


def cmd_doa(args):
    x6 = load6(args.recording)
    cfg = DoaConfig(geometry=ArrayGeometry(offset_deg=args.offset, mirror=args.mirror))
    mics = x6[:, 1:5].astype(np.float32) / 32768.0
    az = []
    for t, est in srp_offline(mics, cfg, win_sec=1.0):
        if est is not None and est.confidence >= 0.15:
            az.append(est.azimuth_deg)
            print(
                f"{t:5.0f}s  {est.azimuth_deg:+6.0f} deg  {direction_label(est.azimuth_deg):<11} conf={est.confidence:.2f}"
            )
    if az:
        mean = float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(az))))))
        print(
            f"mean azimuth {mean:+.0f} deg ({direction_label(mean)}) over {len(az)} s"
        )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("speaker")
    sp.add_argument("genuine")
    sp.add_argument("impostor")
    sp.add_argument(
        "--models", action="store_true", help="also compare candidates/*.onnx"
    )
    dp = sub.add_parser("doa")
    dp.add_argument("recording")
    dp.add_argument("--offset", type=float, default=-15.0)
    dp.add_argument("--mirror", action="store_true")
    args = ap.parse_args()
    if args.cmd == "speaker":
        cmd_speaker(args)
    else:
        cmd_doa(args)


if __name__ == "__main__":
    main()
