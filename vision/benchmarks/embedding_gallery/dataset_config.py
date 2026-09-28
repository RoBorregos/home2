"""Per-object-set config — the ONLY place dataset-specific class names live
in this benchmark. A new object set (new dataset, new competition season,
new robot) almost certainly needs different values here:

  - out_of_gallery_classes: which classes to hold out of gallery_photos/
    entirely, so their crops can serve as genuine "not in the gallery"
    negatives for unknown_rejection_rate. Pick 2-3 classes you don't mind
    losing from the gallery for calibration purposes.
  - hard_negative_classes: classes worth curating into hard_negatives/
    because they're visually close (see the plan: hard negatives must be
    *chosen*, not random). You can guess a first pass from the object list
    (cutlery, same-shape-different-color items, etc.).
  - known_limitation_classes: classes the calibrated gate excludes because
    they ONLY confuse with each other, never with an unrelated class, and
    are already covered by another detector (e.g. yolo_finetuned). This
    CANNOT be guessed ahead of time — it's discovered by running
    report.py/e2e_calibrate.py on the new dataset and reading the
    per-class confusion breakdown in results/*.json, the same way this
    benchmark's own list was built. Start this one empty for a new object
    set; only add a class once you have evidence, not a hunch.

Edit dataset_config.json (not this file, not the scripts that import it) to
adapt to a new object set.
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
