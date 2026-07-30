"""Guard that the fakes still match the real facades.

This replaces the ABC the previous design used: RobotArm and Perception are
plain concrete classes now, so nothing structural stops FakeArm from drifting
out of date and letting the pipeline tests pass against a stale interface.
"""

import inspect

from pick_and_place.robot.arm import RobotArm
from pick_and_place.robot.perception import Perception

from fakes import FakeArm, FakePerception


def _public_api(cls):
    return {
        name
        for name, member in vars(cls).items()
        if not name.startswith("_")
        and (inspect.isfunction(member) or isinstance(member, property))
    }


def test_fake_arm_covers_the_real_arm():
    missing = _public_api(RobotArm) - set(dir(FakeArm))
    assert not missing, f"FakeArm is missing: {sorted(missing)}"


def test_fake_perception_covers_the_real_perception():
    missing = _public_api(Perception) - set(dir(FakePerception))
    assert not missing, f"FakePerception is missing: {sorted(missing)}"


def test_pipelines_do_not_import_ros_clients():
    """The pipelines must go through the facades, not talk ROS directly.

    This is the layering rule the refactor exists to enforce; without a test it
    erodes the first time someone needs "just one more service call".
    """
    import pathlib

    pipelines = pathlib.Path(inspect.getfile(RobotArm)).parent.parent / "pipelines"
    offenders = []
    for path in pipelines.glob("*.py"):
        source = path.read_text()
        for banned in ("ActionClient", "create_client", "create_subscription"):
            if banned in source:
                offenders.append(f"{path.name}: {banned}")
    assert not offenders, f"pipelines must not create ROS clients: {offenders}"
