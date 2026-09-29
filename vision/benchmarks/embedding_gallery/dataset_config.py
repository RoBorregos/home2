"""Per-object-set config — the only place dataset-specific class names live
in this benchmark. Edit dataset_config.json (not this file) to adapt to a
new object set:

  - out_of_gallery_classes: 2-3 classes held out of gallery_photos/ to serve
    as "not in gallery" negatives for unknown-rejection.
  - hard_negative_classes: visually-close classes curated into
    hard_negatives/ — chosen, not random.
  - known_limitation_classes: classes excluded from the recall gate because
    they only confuse each other, never an unrelated class. Can't be
    guessed — start empty, add a class only once report.py/e2e_calibrate.py
    shows evidence in its confusion breakdown.
"""

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
