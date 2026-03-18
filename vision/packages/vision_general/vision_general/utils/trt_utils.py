"""
TensorRT YOLO loading utility for Orin AGX.

Automatically exports YOLO .pt models to TensorRT .engine on first run.
Subsequent runs load the cached engine directly (~2-5x faster inference).
Engine files are device-specific and saved next to the original .pt file.

For persistent caching across Docker restarts, mount a volume to the
model directory (e.g., -v /path/on/host:/workspace/models).
"""

import os
from ultralytics import YOLO


def load_yolo_trt(model_path: str, task: str = "detect", imgsz: int = 640) -> YOLO:
    """Load a YOLO model with automatic TensorRT acceleration.

    Args:
        model_path: Path to .pt model file.
        task: YOLO task type ("detect", "pose", "segment").
        imgsz: Input image size for TensorRT engine.

    Returns:
        YOLO model (TensorRT engine if available, PyTorch fallback).
    """
    engine_path = model_path.replace(".pt", ".engine")

    if os.path.exists(engine_path):
        print(f"[TRT] Loading cached engine: {engine_path}")
        return YOLO(engine_path, task=task)

    if not os.path.exists(model_path):
        print(f"[TRT] Model not found: {model_path}, loading by name")
        model = YOLO(model_path)
    else:
        model = YOLO(model_path)

    try:
        print(f"[TRT] Exporting {model_path} to TensorRT (first run only)...")
        model.export(format="engine", half=True, device=0, imgsz=imgsz)
        print(f"[TRT] Engine saved: {engine_path}")
        return YOLO(engine_path, task=task)
    except Exception as e:
        print(f"[TRT] Export failed ({e}), using PyTorch model")
        return model
