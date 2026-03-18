"""
TensorRT YOLO loading utility for Orin AGX.

Automatically exports YOLO .pt models to TensorRT .engine on first run.
Subsequent runs load the cached engine directly (~2-5x faster inference).

Engine files are saved to TENSORRT_CACHE_DIR (default: /workspace/trt_cache)
which should be a mounted volume for persistence across container restarts.
"""

import os
from ultralytics import YOLO


def _get_engine_path(model_path: str) -> str:
    """Get the engine path, using TENSORRT_CACHE_DIR if set."""
    cache_dir = os.environ.get("TENSORRT_CACHE_DIR")
    engine_name = os.path.basename(model_path).replace(".pt", ".engine")
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        return os.path.join(cache_dir, engine_name)
    return model_path.replace(".pt", ".engine")


def load_yolo_trt(model_path: str, task: str = "detect", imgsz: int = 640) -> YOLO:
    """Load a YOLO model with automatic TensorRT acceleration.

    Args:
        model_path: Path to .pt model file.
        task: YOLO task type ("detect", "pose", "segment").
        imgsz: Input image size for TensorRT engine.

    Returns:
        YOLO model (TensorRT engine if available, PyTorch fallback).
    """
    engine_path = _get_engine_path(model_path)

    if os.path.exists(engine_path):
        print(f"[TRT] Loading cached engine: {engine_path}")
        return YOLO(engine_path, task=task)

    model = YOLO(model_path)

    try:
        print(f"[TRT] Exporting {model_path} to TensorRT (first run only)...")
        model.export(format="engine", half=True, device=0, imgsz=imgsz)
        # ultralytics saves engine next to .pt — move it to cache dir
        local_engine = model_path.replace(".pt", ".engine")
        if local_engine != engine_path and os.path.exists(local_engine):
            os.rename(local_engine, engine_path)
        print(f"[TRT] Engine saved: {engine_path}")
        return YOLO(engine_path, task=task)
    except Exception as e:
        print(f"[TRT] Export failed ({e}), using PyTorch model")
        return model
