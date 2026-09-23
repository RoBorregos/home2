"""MODEL_CONFIGS catalog and ModelRegistry: maps model names to loaded singleton instances."""

import json
import pathlib

# .pt files (and the gallery/ dir) live directly beside this file — same
# place fetch_models.py's sync_detector_models() copies DETECTOR_MODELS to.
MODELS_PATH = str(pathlib.Path(__file__).parent) + "/"

# To add a new YOLO with the same architecture:
#   1. Drop the .pt file beside this one (scripts/detectors/)
#   2. Add one entry here — zero other code changes needed
#
# New model architecture (compatible deps) → new file in models/ + one entry here
# Conflicting deps or exclusive GPU >4 GB → new gRPC container in docker/vision/
#
# Optional "translation" key: a JSON file (resolved against MODELS_PATH) mapping

MODEL_CONFIGS: dict[str, dict] = {
    "yolo_finetuned": {
        "filename": "robocup2026_v1.pt",
        "type": "yolo",
        "conf": 0.6,
        "translation": "robocup2026_translation.json",
        "use_trt": True,
    },
    "yolo_generic": {
        "filename": "yolo26n.pt",
        "type": "yolo",
        "conf": 0.5,
        "use_trt": True,
    },
    "zero_shot": {"filename": "yoloe-11l-seg.pt", "type": "yolo_e", "conf": 0.25},
    # Few-shot object recognition (add an object from photos, no retraining).
    # Not yet in ObjectDetect2D's `models:` param — see
    # vision/benchmarks/embedding_gallery/README.md for the gate this passed
    # and vision/.../plans docs for why this stays opt-in until validated on
    # the Orin (latency, iou_deduplicate ordering, GPU footprint).
    "embedding_box_proposer": {
        "filename": "yoloe-11l-seg-pf.pt",
        "type": "yolo_e",
        "conf": 0.10,
    },
    "embedding_gallery": {
        "type": "embedding",
        "backbone": "vit_base_patch14_dinov2.lvd142m",
        "box_model": "embedding_box_proposer",
        "gallery_dir": "gallery",
        "translation": "robocup2026_translation.json",
    },
}


def _load_translation(config: dict) -> dict[str, str]:
    """Load the optional label-translation map for a model config."""
    filename = config.get("translation")
    if not filename:
        return {}
    path = MODELS_PATH + filename
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[ModelRegistry] translation file not found: {path}")
        return {}


class ModelRegistry:
    _type_registry: dict = {}  # type_name -> class
    _instances: dict = {}  # model_name -> loaded instance

    @classmethod
    def register(cls, type_name: str):
        """Decorator: @ModelRegistry.register()"""

        def decorator(model_cls):
            cls._type_registry[type_name] = model_cls
            return model_cls

        return decorator

    @classmethod
    def get(cls, name: str):
        """Return a loaded singleton for the given model name."""
        if name not in cls._instances:
            if name not in MODEL_CONFIGS:
                raise KeyError(
                    f"Unknown model '{name}'. Add it to MODEL_CONFIGS in registry.py"
                )
            config = MODEL_CONFIGS[name]
            type_name = config["type"]
            if type_name not in cls._type_registry:
                raise KeyError(
                    f"Model type '{type_name}' not registered. "
                    f"Import its module before calling get()."
                )
            instance = cls._type_registry[type_name](name)
            instance.load(config)
            instance.set_translation(_load_translation(config))
            cls._instances[name] = instance
        return cls._instances[name]
