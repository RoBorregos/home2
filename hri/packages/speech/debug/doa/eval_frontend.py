"""Offline comparison of audio front ends on 6-channel ReSpeaker recordings.

For every front end (ch0 = today's input, raw ch1, mic mean, RTF-MVDR, and the
DeepFilterNet variants when the ``df`` package is available) it writes the mono
output as WAV and reports:

* level and a speech/non-speech power ratio (SNR proxy), with one set of VAD
  labels shared by all front ends (computed on the raw mic mean);
* speaker-id separation when --genuine / --impostor recordings are given:
  genuine scores are leave-one-out against a profile built from the other
  genuine segments *of the same front end*, impostor scores against the full
  genuine profile. Reports genuine min/median, impostor max/p95 and d'.

    python3 eval_frontend.py --genuine rec6_mario.wav --impostor rec6_imp.wav \
        --out /tmp/fe_eval [--atten 6] [--no-dfn]
    python3 eval_frontend.py --files right_mario.wav front_mario.wav --out /tmp/fe
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import soundfile as sf

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PKG)

from speech.spatial.denoise import make_dfn_enhance
from speech.spatial.frontend import FrontEndConfig, SpatialFrontEnd
from speech.speaker_id.silero import FRAME_SAMPLES, SileroVAD

SR = 16000
CHUNK = 1024
ASSETS = os.path.join(PKG, "assets")
SILERO = os.path.join(ASSETS, "silero_vad.onnx")
EMBED = os.path.join(ASSETS, "downloads", "speaker_id", "voxceleb_resnet34_LM.onnx")


def load6(path):
    x, sr = sf.read(path, dtype="int16")
    assert sr == SR and x.ndim == 2 and x.shape[1] == 6, path
    return x


def run_frontend(x6, cfg, enhance_fn=None):
    fe = SpatialFrontEnd(SileroVAD(SILERO), cfg, enhance_fn)
    out, labels = [], []
    for s in range(0, x6.shape[0] - CHUNK + 1, CHUNK):
        out.append(fe.process(x6[s : s + CHUNK]))
        labels.append(fe.speech)
    y = np.concatenate(out)
    return np.pad(y, (0, max(0, x6.shape[0] - y.shape[0])))[: x6.shape[0]], labels


def old_chunked_dfn(mono, model, state, df_module, atten):
    """Exactly noise_cancellation.py: 1024-sample chunks, per-chunk peak norm."""
    import scipy.signal
    import torch

    out = []
    for s in range(0, mono.shape[0] - CHUNK + 1, CHUNK):
        a = mono[s : s + CHUNK]
        peak = float(np.max(np.abs(a)))
        if peak == 0:
            out.append(a)
            continue
        a48 = scipy.signal.resample_poly(a / peak, 3, 1).astype(np.float32)
        with torch.no_grad():
            e = df_module.enhance(
                model, state, torch.from_numpy(a48).unsqueeze(0), atten_lim_db=atten
            )
        e16 = scipy.signal.resample_poly(e.cpu().numpy().squeeze(), 1, 3)
        out.append((e16 * peak).astype(np.float32))
    y = np.concatenate(out)
    return np.pad(y, (0, mono.shape[0] - y.shape[0]))


def speech_labels(x6):
    """Per-sample speech mask from Silero on the raw mic mean (+12 dB)."""
    vad = SileroVAD(SILERO)
    mono = np.clip(x6[:, 1:5].astype(np.float32).mean(1) / 32768.0 * 4.0, -1, 1)
    mask = np.zeros(x6.shape[0], bool)
    for s in range(0, mono.shape[0] - FRAME_SAMPLES + 1, FRAME_SAMPLES):
        mask[s : s + FRAME_SAMPLES] = vad(mono[s : s + FRAME_SAMPLES]) >= 0.5
    return mask


def segments(mask, min_sec=1.0, max_sec=3.0, gap_sec=0.4):
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    runs, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > gap_sec * SR:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    out = []
    for a, b in runs:
        n = b - a
        if n < min_sec * SR:
            continue
        step = int(max_sec * SR)
        for s in range(a, b, step):
            if min(b, s + step) - s >= min_sec * SR:
                out.append((s, min(b, s + step)))
    return out


def snr_proxy(y, mask):
    sp = np.mean(y[mask] ** 2) if mask.any() else 0
    ns = np.mean(y[~mask] ** 2) if (~mask).any() else 1e-12
    return 10 * np.log10(sp / max(ns, 1e-12) + 1e-12), 10 * np.log10(
        np.mean(y**2) + 1e-12
    )


def l2(v):
    return v / (np.linalg.norm(v) + 1e-12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--genuine")
    ap.add_argument("--impostor")
    ap.add_argument("--files", nargs="*", default=[])
    ap.add_argument("--out", default="/tmp/fe_eval")
    ap.add_argument("--atten", type=float, default=6.0)
    ap.add_argument("--no-dfn", action="store_true")
    ap.add_argument("--max-sec", type=float, default=0, help="crop inputs (0 = all)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    dfn = None
    if not args.no_dfn:
        try:
            import df as df_module

            model_dir = os.path.join(ASSETS, "downloads", "DeepFilterNet3")
            model, state, _ = df_module.init_df(
                model_base_dir=model_dir, log_level="none", log_file=None
            )
            import torch

            if torch.cuda.is_available():
                model = model.to("cuda")
            dfn = (model, state, df_module)
        except Exception as exc:  # noqa: BLE001
            print(f"DFN unavailable ({exc!r}); skipping DFN front ends")

    def frontends(x6):
        res = {}
        t = time.time()
        res["ch0"] = x6[:, 0].astype(np.float32) / 32768.0
        res["ch1"] = x6[:, 1].astype(np.float32) / 32768.0
        res["mean"], _ = run_frontend(x6, FrontEndConfig(use_mvdr=False))
        res["mvdr"], _ = run_frontend(x6, FrontEndConfig())
        if dfn is not None:
            m, st, mod = dfn
            res["ch0+dfn12_old"] = old_chunked_dfn(res["ch0"], m, st, mod, 12.0)
            fn = make_dfn_enhance(m, st, mod, args.atten)
            res[f"mean+dfn{args.atten:g}"], _ = run_frontend(
                x6, FrontEndConfig(use_mvdr=False), fn
            )
            res[f"mvdr+dfn{args.atten:g}"], _ = run_frontend(x6, FrontEndConfig(), fn)
        print(
            f"  front ends computed in {time.time() - t:.1f}s "
            f"for {x6.shape[0] / SR:.1f}s of audio"
        )
        return res

    def crop(x):
        return x[: int(args.max_sec * SR)] if args.max_sec else x

    # ---------------------------------------------------------------- SNR
    for path in args.files + [p for p in (args.genuine, args.impostor) if p]:
        x6 = crop(load6(path))
        mask = speech_labels(x6)
        print(
            f"\n== {os.path.basename(path)}: {x6.shape[0] / SR:.1f}s, "
            f"speech {mask.mean() * 100:.0f}%"
        )
        res = frontends(x6)
        base = os.path.splitext(os.path.basename(path))[0]
        for name, y in res.items():
            snr, lvl = snr_proxy(y, mask)
            sf.write(os.path.join(args.out, f"{base}__{name}.wav"), y, SR)
            print(
                f"  {name:<16} level {lvl:6.1f} dBFS   speech/non-speech {snr:5.1f} dB"
            )
        if path == args.genuine:
            gen = (res, mask)
        if path == args.impostor:
            imp = (res, mask)

    if not (args.genuine and args.impostor):
        return

    # ----------------------------------------------------------- speaker id
    from speech.speaker_id.embedder import SpeakerEmbedder

    emb = SpeakerEmbedder(EMBED, ["cuda", "cpu"], SR)
    g_res, g_mask = gen
    i_res, i_mask = imp
    g_segs, i_segs = segments(g_mask), segments(i_mask)
    print(f"\nspeaker id: {len(g_segs)} genuine / {len(i_segs)} impostor segments")
    print(f"  {'front end':<16} gen_min gen_med  imp_max imp_p95   d'    sep(min-max)")
    for name in g_res:
        G = np.stack([l2(emb.embed(g_res[name][a:b])) for a, b in g_segs])
        I = np.stack([l2(emb.embed(i_res[name][a:b])) for a, b in i_segs])
        loo = []
        for k in range(G.shape[0]):
            prof = l2(np.delete(G, k, 0).mean(0))
            loo.append(float(G[k] @ prof))
        prof = l2(G.mean(0))
        imp_s = I @ prof
        gs, is_ = np.array(loo), np.array(imp_s)
        d = (gs.mean() - is_.mean()) / np.sqrt(0.5 * (gs.var() + is_.var()) + 1e-12)
        print(
            f"  {name:<16} {gs.min():6.2f} {np.median(gs):6.2f}   {is_.max():6.2f} "
            f"{np.percentile(is_, 95):6.2f}  {d:5.2f}  {gs.min() - is_.max():6.2f}"
        )


if __name__ == "__main__":
    main()
