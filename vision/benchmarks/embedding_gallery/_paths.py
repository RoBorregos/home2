"""Side-effect import: adds detectors/ to sys.path for backbone.py/gallery_matcher.py.
Must be `import _paths`, not `from _paths import ...` — a plain import needs no E402 suppression for imports after it."""

import sys
from pathlib import Path

_DETECTORS_DIR = (
    Path(__file__).resolve().parents[2]
    / "packages"
    / "object_detector_2d"
    / "scripts"
    / "detectors"
)
if str(_DETECTORS_DIR) not in sys.path:
    sys.path.insert(0, str(_DETECTORS_DIR))
