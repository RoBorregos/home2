"""Per-object-set config — the only place dataset-specific class names live.
Edit dataset_config.json (not this file): out_of_gallery_classes (unknown-rejection negatives), hard_negative_classes (curated look-alikes), known_limitation_classes (start empty, add only with confusion-matrix evidence)."""

import json
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).parent / "dataset_config.json"

_EMPTY = {
    "out_of_gallery_classes": set(),
    "hard_negative_classes": set(),
    "known_limitation_classes": set(),
}


def load_dataset_config(path: Path | None = None) -> dict[str, set[str]]:
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        print(f"[dataset_config] {path} not found — starting from empty class sets")
        return dict(_EMPTY)
    data = json.loads(path.read_text())
    return {
        "out_of_gallery_classes": set(data.get("out_of_gallery_classes", [])),
        "hard_negative_classes": set(data.get("hard_negative_classes", [])),
        "known_limitation_classes": set(data.get("known_limitation_classes", [])),
    }
