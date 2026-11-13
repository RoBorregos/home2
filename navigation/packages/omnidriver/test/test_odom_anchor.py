import math

from omnidriver.odrive_dashboard import _reanchor, _se2_mul


def test_reanchor_keeps_pose_after_firmware_reset():
    off, last = (0.3, -0.2, 0.4), (2.0, 1.0, 1.2)
    before = _se2_mul(off, last)
    for raw in [(0.0, 0.0, 0.0), (5.0, -3.0, 2.5)]:
        after = _se2_mul(_reanchor(off, last, raw), raw)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(before, after))
