#!/usr/bin/env python3
"""
Train a custom openwakeword classifier head for a single word.

What this produces is a `.onnx` file that plugs directly into the existing
`kws_oww.py` node: drop it in `hri/packages/speech/assets/oww_retrained/` and
relaunch. The ONNX input signature matches what openwakeword expects for a
wakeword head:

    input:  (batch, 16, 96)  float32  — 16 embedding frames × 96 dims
    output: (batch, 1)       float32  — score in [0, 1]

Feature extraction is done with the SAME two ONNX files openwakeword uses at
inference time (melspectrogram.onnx + embedding_model.onnx, already present
in hri/packages/speech/assets/downloads/), so train/inference preprocessing
is byte-identical.

Pipeline:
  1. Collect positive clips (augmented/<word>/*.wav)
  2. Collect negative clips (augmented/<other_words>/*.wav +
     augmented/negatives/*.wav + augmented/backgrounds/*.wav)
  3. Convert each clip to 16 embedding frames of 96 dims using
     the openwakeword feature models
  4. Train a tiny Conv1D classifier in PyTorch
  5. Export to ONNX with input shape (1, 16, 96)

Usage (from the training/ directory, with the training requirements installed):

    python3 scripts/train_word.py --word yes --epochs 30
    python3 scripts/train_word.py --word no  --epochs 30
    python3 scripts/train_word.py --word stop --epochs 30
    python3 scripts/train_word.py --word frida --epochs 30
"""

import argparse
import glob
import os
import sys
import wave
from typing import List, Tuple

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("error: onnxruntime not installed. pip install onnxruntime", file=sys.stderr)
    sys.exit(1)

try:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset
except ImportError:
    print("error: torch not installed. pip install torch", file=sys.stderr)
    sys.exit(1)


SAMPLE_RATE = 16000
TARGET_SAMPLES = 20480  # 1.28 s
N_EMBED_FRAMES = 16
EMBED_DIM = 96

# Constants for the openwakeword feature models (these match what the
# upstream library does internally — do not change).
MEL_WINDOW_SAMPLES = 1280  # 80 ms hop for melspectrogram model
EMBED_MEL_WINDOW = 76  # mel frames per embedding window
EMBED_MEL_STRIDE = 8  # mel frames between successive embedding windows

_THIS_FILE = os.path.abspath(__file__)
DEFAULT_DATA_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(_THIS_FILE), "..", "data")
)
DEFAULT_ASSETS_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(_THIS_FILE), "..", "..", "assets")
)
DEFAULT_OUTPUT_DIR = os.path.join(DEFAULT_ASSETS_ROOT, "oww_retrained")


# ---------------------------------------------------------------------------
# Feature extraction — replicates openwakeword.utils.AudioFeatures
# ---------------------------------------------------------------------------


class FeatureExtractor:
    """Wraps the two openwakeword feature ONNX models.

    Given a 1.28 s int16 audio clip, returns a (16, 96) float32 feature
    array — the exact tensor kws_oww feeds to the classifier head at
    inference time.
    """

    def __init__(self, assets_root: str):
        downloads = os.path.join(assets_root, "downloads")
        mel_path = os.path.join(downloads, "melspectrogram.onnx")
        emb_path = os.path.join(downloads, "embedding_model.onnx")
        for p in (mel_path, emb_path):
            if not os.path.isfile(p):
                raise FileNotFoundError(
                    f"Missing feature model: {p}\n"
                    "Run kws_oww.py once to auto-download feature models, or "
                    "copy melspectrogram.onnx + embedding_model.onnx into "
                    f"{downloads} manually."
                )

        providers = ["CPUExecutionProvider"]
        self.mel = ort.InferenceSession(mel_path, providers=providers)
        self.emb = ort.InferenceSession(emb_path, providers=providers)
        self.mel_input = self.mel.get_inputs()[0].name
        self.emb_input = self.emb.get_inputs()[0].name

    def _mel_from_audio(self, audio_int16: np.ndarray) -> np.ndarray:
        """Compute mel frames for an int16 audio array."""
        x = audio_int16.astype(np.float32)[None, :]  # (1, n_samples)
        mel = self.mel.run(None, {self.mel_input: x})[0]
        # openwakeword normalises: mel / 10 + 2
        mel = mel.squeeze() / 10.0 + 2.0
        # Shape: (n_mel_frames, 32)
        return mel

    def _embed_from_mel(self, mel: np.ndarray) -> np.ndarray:
        """Slide a 76-frame window with stride 8 over the mel spectrogram."""
        n_frames = mel.shape[0]
        if n_frames < EMBED_MEL_WINDOW:
            pad = np.zeros((EMBED_MEL_WINDOW - n_frames, mel.shape[1]), dtype=mel.dtype)
            mel = np.concatenate([mel, pad], axis=0)
            n_frames = mel.shape[0]
        windows = []
        for start in range(0, n_frames - EMBED_MEL_WINDOW + 1, EMBED_MEL_STRIDE):
            windows.append(mel[start : start + EMBED_MEL_WINDOW])
        if not windows:
            windows = [mel[:EMBED_MEL_WINDOW]]
        batch = np.stack(windows, axis=0).astype(np.float32)
        # Embedding model expects (batch, 76, 32, 1)
        batch = batch[..., None]
        out = self.emb.run(None, {self.emb_input: batch})[0]
        # Output: (batch, 1, 1, 96) -> (batch, 96)
        return out.reshape(out.shape[0], -1)

    def extract(self, audio_int16: np.ndarray) -> np.ndarray:
        """Return a (16, 96) float32 feature tensor for a 1.28 s clip."""
        if len(audio_int16) < TARGET_SAMPLES:
            audio_int16 = np.pad(audio_int16, (0, TARGET_SAMPLES - len(audio_int16)))
        elif len(audio_int16) > TARGET_SAMPLES:
            audio_int16 = audio_int16[:TARGET_SAMPLES]
        mel = self._mel_from_audio(audio_int16)
        embeds = self._embed_from_mel(mel)
        if embeds.shape[0] >= N_EMBED_FRAMES:
            embeds = embeds[-N_EMBED_FRAMES:]
        else:
            pad = np.zeros(
                (N_EMBED_FRAMES - embeds.shape[0], EMBED_DIM), dtype=np.float32
            )
            embeds = np.concatenate([pad, embeds], axis=0)
        return embeds.astype(np.float32)


# ---------------------------------------------------------------------------
# Dataset building
# ---------------------------------------------------------------------------


def read_wav_int16(path: str) -> np.ndarray:
    with wave.open(path, "rb") as wf:
        if (
            wf.getnchannels() != 1
            or wf.getsampwidth() != 2
            or wf.getframerate() != SAMPLE_RATE
        ):
            raise ValueError(
                f"{path}: expected mono 16 kHz 16-bit (got "
                f"{wf.getnchannels()}ch, {wf.getsampwidth() * 8}-bit, "
                f"{wf.getframerate()} Hz)"
            )
        data = wf.readframes(wf.getnframes())
    return np.frombuffer(data, dtype=np.int16)


def collect_features(
    paths: List[str], extractor: FeatureExtractor, label: int
) -> Tuple[np.ndarray, np.ndarray]:
    feats = []
    for p in paths:
        try:
            audio = read_wav_int16(p)
        except Exception as e:
            print(f"  skip {p}: {e}")
            continue
        feats.append(extractor.extract(audio))
    if not feats:
        return (
            np.zeros((0, N_EMBED_FRAMES, EMBED_DIM), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
        )
    X = np.stack(feats, axis=0)
    y = np.full((X.shape[0],), float(label), dtype=np.float32)
    return X, y


def build_dataset(
    word: str,
    data_root: str,
    extractor: FeatureExtractor,
) -> Tuple[np.ndarray, np.ndarray]:
    aug_root = os.path.join(data_root, "augmented")
    if not os.path.isdir(aug_root):
        raise FileNotFoundError(
            f"No augmented data found at {aug_root}. Run augment_clips.py first."
        )

    pos_dir = os.path.join(aug_root, word)
    pos_paths = sorted(glob.glob(os.path.join(pos_dir, "*.wav")))
    if not pos_paths:
        raise FileNotFoundError(
            f"No positive clips for '{word}' at {pos_dir}. "
            "Record samples with record_samples.py and run augment_clips.py."
        )

    neg_paths: List[str] = []
    for name in sorted(os.listdir(aug_root)):
        sub = os.path.join(aug_root, name)
        if not os.path.isdir(sub) or name == word:
            continue
        neg_paths.extend(sorted(glob.glob(os.path.join(sub, "*.wav"))))

    if not neg_paths:
        raise FileNotFoundError(
            "No negative clips found. Record 'negatives' and/or 'backgrounds' "
            "with record_samples.py and augment them."
        )

    print(f"  {len(pos_paths)} positive, {len(neg_paths)} negative augmented clips")
    print("  extracting features (this takes a minute the first time)...")
    Xp, yp = collect_features(pos_paths, extractor, 1)
    Xn, yn = collect_features(neg_paths, extractor, 0)
    print(f"  features: positives={Xp.shape}, negatives={Xn.shape}")
    X = np.concatenate([Xp, Xn], axis=0)
    y = np.concatenate([yp, yn], axis=0)
    return X, y


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


class WakewordHead(nn.Module):
    """Tiny classifier that maps (B, 16, 96) -> (B, 1) sigmoid score."""

    def __init__(self, hidden: int = 64):
        super().__init__()
        # Treat 16 as the time axis, 96 as input channels.
        self.conv1 = nn.Conv1d(EMBED_DIM, hidden, kernel_size=5, padding=2)
        self.conv2 = nn.Conv1d(hidden, hidden, kernel_size=5, padding=2)
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.fc = nn.Linear(hidden, 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, 16, 96) -> (B, 96, 16)
        x = x.transpose(1, 2)
        x = torch.relu(self.conv1(x))
        x = torch.relu(self.conv2(x))
        x = self.pool(x).squeeze(-1)
        return torch.sigmoid(self.fc(x))


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------


def train(
    X: np.ndarray,
    y: np.ndarray,
    epochs: int,
    batch_size: int,
    lr: float,
    val_split: float,
    device: str,
) -> WakewordHead:
    rng = np.random.default_rng(1337)
    perm = rng.permutation(len(X))
    X, y = X[perm], y[perm]

    n_val = max(1, int(len(X) * val_split))
    Xv, yv = X[:n_val], y[:n_val]
    Xt, yt = X[n_val:], y[n_val:]

    # Balance loss using positive weight.
    n_pos = float((yt == 1).sum())
    n_neg = float((yt == 0).sum())
    pos_weight = max(1.0, n_neg / max(1.0, n_pos))
    print(f"  train pos={int(n_pos)} neg={int(n_neg)} pos_weight={pos_weight:.2f}")

    train_ds = TensorDataset(torch.from_numpy(Xt), torch.from_numpy(yt))
    val_ds = TensorDataset(torch.from_numpy(Xv), torch.from_numpy(yv))
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

    model = WakewordHead().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    # BCE with class balancing (applied as sample weights).
    bce = nn.BCELoss(reduction="none")

    best_val = float("inf")
    best_state = None
    for epoch in range(1, epochs + 1):
        model.train()
        total = 0.0
        n = 0
        for xb, yb in train_loader:
            xb = xb.to(device)
            yb = yb.to(device)
            opt.zero_grad()
            pred = model(xb).squeeze(-1)
            weight = torch.where(
                yb > 0.5,
                torch.full_like(yb, pos_weight),
                torch.ones_like(yb),
            )
            loss = (bce(pred, yb) * weight).mean()
            loss.backward()
            opt.step()
            total += float(loss.item()) * xb.size(0)
            n += xb.size(0)
        train_loss = total / max(1, n)

        model.eval()
        vt = 0.0
        vn = 0
        correct = 0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb = xb.to(device)
                yb = yb.to(device)
                pred = model(xb).squeeze(-1)
                loss = bce(pred, yb).mean()
                vt += float(loss.item()) * xb.size(0)
                vn += xb.size(0)
                correct += int(((pred > 0.5) == (yb > 0.5)).sum().item())
        val_loss = vt / max(1, vn)
        val_acc = correct / max(1, vn)

        print(
            f"  epoch {epoch:3d}  train_loss={train_loss:.4f}  "
            f"val_loss={val_loss:.4f}  val_acc={val_acc:.3f}"
        )
        if val_loss < best_val:
            best_val = val_loss
            best_state = {
                k: v.detach().cpu().clone() for k, v in model.state_dict().items()
            }

    if best_state is not None:
        model.load_state_dict(best_state)
    return model


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------


def export_onnx(model: WakewordHead, out_path: str) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    model.eval().cpu()
    dummy = torch.zeros(1, N_EMBED_FRAMES, EMBED_DIM, dtype=torch.float32)
    torch.onnx.export(
        model,
        dummy,
        out_path,
        input_names=["input"],
        output_names=["score"],
        opset_version=14,
        dynamic_axes={"input": {0: "batch"}, "score": {0: "batch"}},
    )
    print(f"  exported {out_path}")

    # Sanity check: load with onnxruntime and run a zero input.
    sess = ort.InferenceSession(out_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0].name
    out = sess.run(None, {inp: dummy.numpy()})[0]
    print(
        f"  sanity: input {dummy.shape} -> output {out.shape} (value={out.flatten()[0]:.3f})"
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--word", required=True, help="Word to train (e.g. yes).")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.15)
    parser.add_argument(
        "--data-root",
        default=DEFAULT_DATA_ROOT,
        help="Training data root (contains augmented/).",
    )
    parser.add_argument(
        "--assets-root",
        default=DEFAULT_ASSETS_ROOT,
        help="Speech assets root (contains downloads/ with feature ONNX models).",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Where to write the trained .onnx (default: assets/oww_retrained).",
    )
    parser.add_argument(
        "--device",
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="torch device (cpu or cuda).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print(f"== Training '{args.word}' ==")
    print(f"  device={args.device}")
    print(f"  data_root={args.data_root}")
    print(f"  assets_root={args.assets_root}")
    print(f"  output_dir={args.output_dir}")

    extractor = FeatureExtractor(args.assets_root)
    X, y = build_dataset(args.word, args.data_root, extractor)
    if len(X) < 20:
        print(
            "error: not enough training samples. Record more raw clips and "
            "increase --variants when augmenting.",
            file=sys.stderr,
        )
        sys.exit(1)

    model = train(
        X,
        y,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        val_split=args.val_split,
        device=args.device,
    )

    out_path = os.path.join(args.output_dir, f"{args.word}.onnx")
    export_onnx(model, out_path)
    print("done.")


if __name__ == "__main__":
    main()
