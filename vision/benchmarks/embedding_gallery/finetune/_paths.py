"""Side-effect import: adds detectors/, core/ and calibration/ to sys.path so
finetune scripts can import backbone.py/gallery_matcher.py and reuse core/report.py, calibration/e2e_calibrate.py."""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]  # embedding_gallery/
_DETECTORS_DIR = (
    _ROOT.parent.parent / "packages" / "object_detector_2d" / "scripts" / "detectors"
)
for _p in (_DETECTORS_DIR, _ROOT / "core", _ROOT / "calibration"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
