"""Self-check for the cmd_vel staleness cutoff in _send_control_cmd.

_send_control_cmd re-streams the last cached velocity at 1/tx_period forever, so
a nav2 crash mid-trajectory used to leave the base driving on the last command it
ever published. The cutoff zeroes cmd_vel-sourced velocities that went stale, and
must NOT touch web-UI velocities (those latch on purpose and have their own
on-disconnect zeroing).

Runs without a ROS graph: the method is called unbound against a stub.
"""
import time
from types import SimpleNamespace

from omnidriver.odrive_dashboard import ODriveDashboardNode


def _stub(*, age, is_cmd_vel, vx=0.5, timeout=0.5):
    """Minimal stand-in for the node: only what _send_control_cmd touches."""
    params = {'cmd_vel_timeout': timeout, 'follow_base_yaw_enabled': False}
    sent = []
    return SimpleNamespace(
        tx_vx=vx, tx_vy=0.0, tx_wz=0.0, tx_count=0,
        _cmd_vel_time=time.monotonic() - age,
        _cmd_vel_is_source=is_cmd_vel,
        _cmd_vel_stale_logged=False,
        _follow_base_yaw=0.0, _follow_base_yaw_time=0.0,
        get_parameter=lambda n: SimpleNamespace(value=params[n]),
        get_logger=lambda: SimpleNamespace(warn=lambda _m: None),
        _serial_write=sent.append,
        sent=sent,
    )


def _run(stub):
    ODriveDashboardNode._send_control_cmd(stub)
    return stub.sent[-1]


def test_fresh_cmd_vel_passes_through():
    assert _run(_stub(age=0.0, is_cmd_vel=True)) == '1 0.5000 0.0000 0.0000'


def test_stale_cmd_vel_is_zeroed():
    stub = _stub(age=2.0, is_cmd_vel=True)
    assert _run(stub) == '1 0.0000 0.0000 0.0000'
    assert stub.tx_vx == 0.0
    assert stub._cmd_vel_stale_logged


def test_web_ui_velocity_is_never_aged_out():
    # The web UI latches deliberately -- a stale timestamp must not zero it.
    assert _run(_stub(age=99.0, is_cmd_vel=False)) == '1 0.5000 0.0000 0.0000'


def test_timeout_zero_disables_the_cutoff():
    assert _run(_stub(age=99.0, is_cmd_vel=True, timeout=0.0)) \
        == '1 0.5000 0.0000 0.0000'


def test_already_stopped_stays_quiet():
    # Nothing to zero, so no warning is latched.
    stub = _stub(age=99.0, is_cmd_vel=True, vx=0.0)
    assert _run(stub) == '1 0.0000 0.0000 0.0000'
    assert not stub._cmd_vel_stale_logged
