"""Names and locations shared by gallery_build, the matcher, the registry and fetch_models.

Pure strings and one helper: keep it free of heavy imports.
"""

import os
from pathlib import Path

# Built gallery; sits beside detectors/registry.py at runtime.
GALLERY_DIRNAME = "gallery"
MANIFEST_NAME = "manifest.json"
PHOTOS_DIRNAME = "gallery_photos"  # one folder of enrollment photos per object
CROPS_DIRNAME = "_crops"  # the crop taken from each photo, kept for review
PHOTO_EXTENSIONS = (".jpg", ".jpeg", ".png")  # matched case-insensitively

DEFAULT_TENSORRT_CACHE_DIR = "/workspace/trt_cache"


def tensorrt_cache_dir() -> Path:
    """Persistent cache mount (TENSORRT_CACHE_DIR): engines, HF weights and the built gallery."""
    return Path(os.environ.get("TENSORRT_CACHE_DIR", DEFAULT_TENSORRT_CACHE_DIR))
