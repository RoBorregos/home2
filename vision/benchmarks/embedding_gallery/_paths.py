"""Side-effect import: adds detectors/ to sys.path so report.py, e2e_eval.py
and e2e_calibrate.py can import backbone.py/gallery_matcher.py from there
without installing this benchmark as a package.

`import _paths` (not `from _paths import ...`) is what triggers this — as
a plain import, it doesn't break the "imports before other code" rule the
way a sys.path.insert() call or a setup function call would, so the real
imports after it need no E402 suppression."""

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
