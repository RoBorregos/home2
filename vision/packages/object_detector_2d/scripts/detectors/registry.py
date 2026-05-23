"""MODEL_CONFIGS catalog and ModelRegistry: maps model names to loaded singleton instances."""

import pathlib

# .pt files live in scripts/models/ alongside this file
MODELS_PATH = str(pathlib.Path(__file__).parent) + "/"

# To add a new YOLO with the same architecture:
#   1. Drop the .pt file in scripts/models/
#   2. Add one entry here — zero other code changes needed
#
# New model architecture (compatible deps) → new file in models/ + one entry here
# Conflicting deps or exclusive GPU >4 GB → new gRPC container in docker/vision/
MODEL_CONFIGS: dict[str, dict] = {
    "yolo_finetuned": {"filename": "abril9.pt", "type": "yolo", "conf": 0.6},
    "yolo_generic": {"filename": "yolo26n.pt", "type": "yolo", "conf": 0.5},
    "cutlery": {"filename": "cutlery.pt", "type": "yolo", "conf": 0.3},
    "zero_shot": {"filename": "yoloe-11l-seg.pt", "type": "yolo_e", "conf": 0.25},
}


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
            cls._instances[name] = instance
        return cls._instances[name]
