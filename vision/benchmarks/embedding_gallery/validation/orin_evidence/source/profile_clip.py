#!/usr/bin/env python3
"""Profile CLIP with the production gallery resident; run inside vision on Orin."""

import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np


VISION = Path(__file__).resolve().parents[2]
for package in ("vision_general", "object_detector_2d"):
    sys.path.insert(0, str(VISION / "packages" / package / "scripts"))


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def summary(samples):
    return {
        "mean_ms": float(np.mean(samples)),
        "p50_ms": float(np.percentile(samples, 50)),
        "p95_ms": float(np.percentile(samples, 95)),
        "samples_ms": samples,
    }


def measure(torch, operation, iterations):
    samples = []
    for _ in range(iterations):
        torch.cuda.synchronize()
        start = time.perf_counter()
        operation()
        torch.cuda.synchronize()
        samples.append((time.perf_counter() - start) * 1000)
    return summary(samples)


def memory(torch):
    torch.cuda.synchronize()
    free, total = torch.cuda.mem_get_info()
    meminfo = dict(
        line.split(":", 1) for line in Path("/proc/meminfo").read_text().splitlines()
    )
    return {
        "cuda_free_bytes": free,
        "cuda_total_bytes": total,
        "torch_allocated_bytes": torch.cuda.memory_allocated(),
        "torch_reserved_bytes": torch.cuda.memory_reserved(),
        "system_available_bytes": int(meminfo["MemAvailable"].split()[0]) * 1024,
    }


def command_output(command):
    try:
        return subprocess.check_output(
            command, text=True, stderr=subprocess.STDOUT, timeout=10
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--image", type=Path, required=True, help="Representative RGB crop"
    )
    parser.add_argument("--text", default="a red cup")
    parser.add_argument("--batch-size", type=positive_int, default=8)
    parser.add_argument("--warmup", type=positive_int, default=5)
    parser.add_argument("--iterations", type=positive_int, default=30)
    parser.add_argument(
        "--power-mode",
        required=True,
        help="Record the verified Orin power/clock settings",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    import torch
    from PIL import Image
    from utils.models.image_embedder import ImageEmbedder

    if not torch.cuda.is_available():
        parser.error("CUDA is required; CPU timing cannot satisfy the Orin measurement")

    # Importing detectors registers its implementations. This loads the same
    # YOLOE, DINOv2/TRT, gallery and settings as the production node.
    from detectors.registry import MODEL_CONFIGS, ModelRegistry

    with Image.open(args.image) as source:
        crop = source.convert("RGB")
    crops = [crop] * args.batch_size
    texts = [args.text] * args.batch_size
    torch.cuda.init()
    snapshots = {"before_gallery": memory(torch)}
    gallery = ModelRegistry.get("embedding_gallery")
    if not gallery.gallery.thresholds:
        parser.error(
            "A populated production gallery is required for meaningful measurements"
        )
    if gallery.embedder._session is None:
        parser.error(
            "The production gallery GPU session is unavailable; refusing CPU fallback"
        )
    providers = gallery.embedder._session.get_providers()
    if not any(
        p in providers for p in ("TensorrtExecutionProvider", "CUDAExecutionProvider")
    ):
        parser.error("The production gallery has no GPU execution provider")

    # Explicit crop inference builds/exercises the gallery backend even when
    # the proposer finds no boxes. The proposer remains loaded throughout.
    def gallery_batch():
        return gallery.gallery.match_batch(gallery.embedder.embed_batch(crops))

    for _ in range(args.warmup):
        gallery_batch()
    snapshots["gallery_warm"] = memory(torch)
    baseline = measure(torch, gallery_batch, args.iterations)
    clip = ImageEmbedder("clip:ViT-B/32").load()
    for _ in range(args.warmup):
        clip.embed_batch(crops, normalize=True)
        clip.embed_text(texts, normalize=True)
    snapshots["gallery_and_clip_warm"] = memory(torch)
    torch.cuda.reset_peak_memory_stats()
    timings = {
        "gallery_crops_before_clip": baseline,
        "gallery_crops_with_clip_resident": measure(
            torch, gallery_batch, args.iterations
        ),
        "clip_images_with_gallery_resident": measure(
            torch, lambda: clip.embed_batch(crops, normalize=True), args.iterations
        ),
        "clip_text_with_gallery_resident": measure(
            torch, lambda: clip.embed_text(texts, normalize=True), args.iterations
        ),
    }
    snapshots["after_measurement"] = memory(torch)
    device_tree = Path("/proc/device-tree/model")
    report = {
        "model": clip.model_id,
        "dim": clip.dim,
        "batch_size": args.batch_size,
        "crop_size": crop.size,
        "clip_input_size": int(clip._model.visual.input_resolution),
        "gallery_input_size": gallery.embedder._input_size,
        "image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
        "text": args.text,
        "normalize": True,
        "warmup": args.warmup,
        "iterations": args.iterations,
        "machine": platform.machine(),
        "board": device_tree.read_text().rstrip("\x00")
        if device_tree.exists()
        else None,
        "gpu": torch.cuda.get_device_name(),
        "power_mode": args.power_mode,
        "nvpmodel_query": command_output(["nvpmodel", "-q"]),
        "git_head": command_output(["git", "-C", str(VISION), "rev-parse", "HEAD"]),
        "git_status": command_output(["git", "-C", str(VISION), "status", "--short"]),
        "versions": {
            name: version(name)
            for name in ("torch", "torchvision", "clip", "timm", "numpy")
        },
        "cuda_version": torch.version.cuda,
        "gallery_config": MODEL_CONFIGS["embedding_gallery"],
        "gallery_objects": len(gallery.gallery.thresholds),
        "gallery_providers": providers,
        "memory": snapshots,
        "torch_peak_allocated_bytes": torch.cuda.max_memory_allocated(),
        "torch_peak_reserved_bytes": torch.cuda.max_memory_reserved(),
        "timings": timings,
        "notes": [
            "Wall time includes preprocessing/tokenization, transfer and output conversion.",
            "Models are resident together; calls are sequential, not concurrent.",
            "Gallery timings cover crop embedding and matching, not box proposal or ROS.",
            "PyTorch counters exclude TensorRT/ORT allocations; CUDA free and system memory are global, affected by other processes.",
            "Jetson shares system/GPU memory; do not add these counters together.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Report: {args.output}")


if __name__ == "__main__":
    main()
