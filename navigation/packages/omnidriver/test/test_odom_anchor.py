import math

from omnidriver.odrive_dashboard import _anchor, _reanchor, _se2_mul


def _close(a, b):
    return all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(a, b))


def test_hold_pins_pose_then_motion_passes_through():
    off, last = (0.3, -0.2, 0.4), (2.0, 1.0, 1.2)
    pinned = _se2_mul(off, last)
    # Firmware reset, then the EKF drifts while it boots: the pose must not move.
    for raw in [(0.0, 0.0, 0.0), (0.02, -0.01, 0.05), (0.05, 0.03, 0.12)]:
        off = _anchor(off, last, raw, True, 0.5, 0.7)
        assert _close(_se2_mul(off, raw), pinned)
        last = raw
    # Released: a small step is real motion and keeps the offset.
    step = (last[0] + 0.01, last[1], last[2] + 0.01)
    assert _anchor(off, last, step, False, 0.5, 0.7) is off
    assert not _close(_se2_mul(off, step), pinned)
    # Released: a step no base can do in one sample is re-anchored.
    assert _close(_se2_mul(_anchor(off, last, (9.0, 9.0, 0.0), False, 0.5, 0.7),
                           (9.0, 9.0, 0.0)), pinned)


def test_reanchor_keeps_pose_after_firmware_reset():
    off, last = (0.3, -0.2, 0.4), (2.0, 1.0, 1.2)
    before = _se2_mul(off, last)
    for raw in [(0.0, 0.0, 0.0), (5.0, -3.0, 2.5)]:
        after = _se2_mul(_reanchor(off, last, raw), raw)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(before, after))
