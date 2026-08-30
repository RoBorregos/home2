"""Failure taxonomy shared by every manipulation pipeline.

One recovery policy per failure class, decided by the caller.
"""


class PickAttemptFailed(Exception):
    """This candidate did not work; try the next one."""


class PickAborted(Exception):
    """E-stop or goal cancellation: stop immediately and leave the arm safe."""


class PickHardwareError(Exception):
    """A mode switch or required service failed.

    Further attempts would run against a robot in an unknown state, so the
    caller stops rather than retrying.
    """
