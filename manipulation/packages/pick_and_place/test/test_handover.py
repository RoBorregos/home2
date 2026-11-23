"""The take-from-hand pipeline: geometry, call order and the safety rules.

Runs against FakeArm and needs the ROS message packages, so it only runs inside
the container. Each safety rule in ``pipelines/handover.py`` has a test here.
"""

import asyncio
import threading
from types import SimpleNamespace

import numpy as np
import pytest
from fakes import FakeArm, FakeLogger
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import GoalResponse
from scipy.spatial.transform import Rotation

from pick_and_place.manipulation_core import ManipulationCore
from pick_and_place.pipelines import follow as follow_pipeline
from pick_and_place.pipelines import handover
from pick_and_place.pipelines.errors import PickAborted
from pick_and_place.robot import arm as arm_module
from pick_and_place.robot.arm import RobotArm
from pick_and_place.robot.geometry import approach_axis

WRIST = (0.5, 0.0, 1.0)  # base_link, in front of the robot at chest height


def arm_base_pose(yaw=np.pi / 2):
    """link_base in base_link, mounted 0.34 m up and yawed as on FRIDA."""
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.005
    pose.pose.position.z = 0.34
    q = Rotation.from_euler("z", yaw).as_quat()
    pose.pose.orientation.x, pose.pose.orientation.y = float(q[0]), float(q[1])
    pose.pose.orientation.z, pose.pose.orientation.w = float(q[2]), float(q[3])
    return pose


def make_arm(**kwargs):
    kwargs.setdefault("link_pose", arm_base_pose())
    return FakeArm(**kwargs)


def wrist_point(xyz=WRIST, frame="base_link"):
    point = PointStamped()
    point.header.frame_id = frame
    point.point.x, point.point.y, point.point.z = (float(v) for v in xyz)
    return point


def run(arm, **kwargs):
    request = handover.TakeFromHandRequest(
        hand_point=kwargs.pop("point", wrist_point()), **kwargs
    )
    return handover.execute(arm, request)


MOTION = {
    "open_gripper",
    "close_gripper",
    "move_to_pregrasp",
    "move_to_pose",
    "cartesian_approach",
    "wait_for_payload",
}


# ============================================================================
# request
# ============================================================================


def test_request_fills_the_defaults_for_unset_values():
    request = handover.TakeFromHandRequest(wrist_point(), -1.0, 0.0)
    assert request.grasp_below_hand == handover.GRASP_BELOW_WRIST
    assert request.release_timeout == handover.RELEASE_TIMEOUT

    custom = handover.TakeFromHandRequest(wrist_point(), 0.1, 3.0)
    assert (custom.grasp_below_hand, custom.release_timeout) == (0.1, 3.0)


# ============================================================================
# geometry
# ============================================================================


def test_handle_point_is_below_the_wrist_never_on_it():
    np.testing.assert_allclose(handover.handle_point(WRIST, 0.08), (0.5, 0.0, 0.92))
    for below in (0.0, -0.05):
        with pytest.raises(ValueError):
            handover.handle_point(WRIST, below)


@pytest.mark.parametrize(
    "target, reachable",
    [
        ((0.5, 0.0, 0.9), True),
        ((0.1, 0.0, 0.9), False),  # too close to the arm base
        ((0.8, 0.0, 0.9), False),  # too far
        ((0.5, 0.0, 0.2), False),  # too low
        ((0.5, 0.0, 1.5), False),  # too high
        ((0.0, 0.5, 0.9), True),  # to the side, same distance
    ],
)
def test_within_reach(target, reachable):
    assert (
        handover.within_reach(np.array(target), np.array([0.0, 0.0, 0.34])) is reachable
    )


def test_approach_is_horizontal_from_the_arm_base_and_turned_by_yaw():
    target, base = np.array([0.5, 0.0, 0.9]), np.array([0.0, 0.0, 0.34])
    np.testing.assert_allclose(
        handover.approach_direction(target, base, 0.0), (1, 0, 0)
    )
    turned = handover.approach_direction(target, base, np.pi / 2)
    np.testing.assert_allclose(turned, (0, 1, 0), atol=1e-9)
    with pytest.raises(ValueError):
        handover.approach_direction(np.array([0.0, 0.0, 0.9]), base, 0.0)


@pytest.mark.parametrize("yaw_deg", handover.APPROACH_YAWS_DEG)
def test_fingers_close_horizontally_across_the_approach(yaw_deg):
    approach = handover.approach_direction(
        np.array([0.5, 0.2, 0.9]), np.zeros(3), np.radians(yaw_deg)
    )
    matrix = Rotation.from_quat(handover.handle_grasp_quat(approach)).as_matrix()
    np.testing.assert_allclose(matrix[:, 2], approach, atol=1e-9)
    closing = matrix[:, 1]
    # Horizontal, so a vertical handle sits between the fingers.
    assert abs(closing[2]) < 1e-9
    assert abs(np.dot(closing, approach)) < 1e-9


def test_retreat_goes_back_and_up():
    approach = np.array([1.0, 0.0, 0.0])
    retreat = handover.retreat_direction(approach)
    assert np.isclose(np.linalg.norm(retreat), 1.0)
    assert retreat[2] > 0
    assert np.dot(retreat, approach) < 0


def test_grasp_pose_backs_the_grasp_link_off_the_handle():
    target, approach = np.array([0.5, 0.0, 0.92]), np.array([1.0, 0.0, 0.0])
    pose = handover.grasp_pose(target, approach, -0.125)
    position = pose.pose.position
    np.testing.assert_allclose((position.x, position.y, position.z), (0.375, 0.0, 0.92))
    assert pose.header.frame_id == "base_link"
    np.testing.assert_allclose(approach_axis(pose), approach, atol=1e-9)


# ============================================================================
# pipeline
# ============================================================================


def test_happy_path_grips_waits_for_the_release_then_retreats():
    arm = make_arm()
    outcome = run(arm)

    assert (outcome.success, outcome.payload_detected) == (True, True)
    assert arm.calls == [
        "to_frame:base_link",
        "link_pose:link_base",
        "open_gripper",
        "endpoint_self_collides",
        "move_to_pregrasp",
        "cartesian_approach",
        "close_gripper",
        "wait_for_payload",
        "cartesian_approach",
    ]
    assert arm.contexts == ["take_from_hand"]
    assert arm.payload_waits == [
        (
            handover.PAYLOAD_EFFORT_DELTA,
            handover.RELEASE_TIMEOUT,
            list(handover.PAYLOAD_JOINTS),
        )
    ]
    approach, distance, speed = arm.approaches[0]
    assert (distance, speed) == (handover.PREGRASP_DISTANCE, handover.APPROACH_SPEED)
    # The approach is +x in base_link; link_base is yawed +90 deg, so -y there.
    np.testing.assert_allclose(approach, (0, -1, 0), atol=1e-9)
    retreat = np.asarray(arm.approaches[1][0])
    assert retreat[2] > 0 and retreat[1] > 0  # up and back towards the robot


def test_the_target_is_below_the_wrist_at_every_step():
    arm = make_arm()
    run(arm, grasp_below_hand=0.1)

    for pose in arm.poses["endpoint_self_collides"] + arm.poses["move_to_pregrasp"]:
        assert pose.pose.position.z == pytest.approx(WRIST[2] - 0.1)


def test_a_failed_pre_grasp_tries_the_next_yaw():
    arm = make_arm(pregrasp_results=[False, True])
    outcome = run(arm)

    assert outcome.success
    assert arm.calls.count("move_to_pregrasp") == 2
    first, second = (approach_axis(p) for p in arm.poses["move_to_pregrasp"])
    yaw = np.degrees(np.arctan2(second[1], second[0]) - np.arctan2(first[1], first[0]))
    assert yaw == pytest.approx(handover.APPROACH_YAWS_DEG[1])


def test_a_self_colliding_yaw_is_skipped_without_moving():
    arm = make_arm(self_collides=[True, False])
    outcome = run(arm)

    assert outcome.success
    assert arm.calls.count("endpoint_self_collides") == 2
    assert arm.calls.count("move_to_pregrasp") == 1


def test_no_reachable_approach_fails_without_approaching():
    yaws = len(handover.APPROACH_YAWS_DEG)
    arm = make_arm(pregrasp_results=[False] * yaws)
    outcome = run(arm)

    assert not outcome.success
    assert arm.calls.count("move_to_pregrasp") == yaws
    assert "cartesian_approach" not in arm.calls
    assert "close_gripper" not in arm.calls


def test_a_blocked_approach_opens_backs_out_the_same_line_and_fails():
    arm = make_arm(descent_results=[False, True])
    outcome = run(arm)

    assert (outcome.success, outcome.payload_detected) == (False, False)
    assert arm.calls[-3:] == [
        "cartesian_approach",
        "open_gripper",
        "cartesian_approach",
    ]
    # No other angle towards the person, and no grasp.
    assert arm.calls.count("move_to_pregrasp") == 1
    assert "close_gripper" not in arm.calls
    assert "wait_for_payload" not in arm.calls
    forward, back = (np.asarray(a[0]) for a in arm.approaches)
    np.testing.assert_allclose(back, -forward)


def test_no_retreat_while_the_person_still_holds_the_bag():
    arm = make_arm(payload_results=[False])
    outcome = run(arm)

    assert (outcome.success, outcome.payload_detected) == (True, False)
    assert arm.calls[-1] == "wait_for_payload"
    assert arm.calls.count("cartesian_approach") == 1


def test_the_release_timeout_reaches_the_payload_wait():
    arm = make_arm()
    run(arm, release_timeout=3.0)
    assert arm.payload_waits[0][1] == 3.0


@pytest.mark.parametrize(
    "xyz", [(1.2, 0.0, 1.0), (0.1, 0.0, 1.0), (0.5, 0.0, 2.0), (0.5, 0.0, 0.3)]
)
def test_out_of_reach_fails_without_moving(xyz):
    arm = make_arm()
    outcome = run(arm, point=wrist_point(xyz))

    assert not outcome.success
    assert not MOTION & set(arm.calls)


def test_a_tf_failure_fails_without_moving():
    arm = make_arm(tf_fails=True)
    outcome = run(arm, point=wrist_point(frame="zed_left_camera_optical_frame"))

    assert not outcome.success
    assert arm.calls == ["to_frame:base_link"]


def test_no_arm_base_tf_fails_without_moving():
    arm = FakeArm(link_pose=None)
    outcome = run(arm)

    assert not outcome.success
    assert not MOTION & set(arm.calls)


def test_abort_propagates_before_any_motion():
    arm = make_arm(abort_after=0)
    with pytest.raises(PickAborted):
        run(arm)
    assert not MOTION & set(arm.calls)


def test_abort_after_the_grip_skips_the_retreat():
    # Checks: open, pre-grasp, approach, then the one before the retreat.
    arm = make_arm(abort_after=3)
    with pytest.raises(PickAborted):
        run(arm)
    assert arm.calls[-1] == "wait_for_payload"


# ============================================================================
# RobotArm.wait_for_payload
# ============================================================================


class _Clock:
    """Fake time for the arm module; each sleep is one /joint_states reading."""

    def __init__(self, efforts):
        self.now = 0.0
        self.ticks = 0
        self._efforts = efforts
        self.arm = None

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.ticks += 1
        if self.arm is not None and self.arm._latest_joint_state is not None:
            self.arm._latest_joint_state.effort = self._efforts(self.ticks)


# Joints listed out of order, so a lookup by index would read the wrong ones.
NAMES = ["joint3", "joint1", "joint2", "joint6", "joint5", "joint4"]
BASELINE_TICKS = 10  # _sample_effort_baseline's readings


def efforts(load_joint="joint3", delta=2.0, start=5, end=None):
    """Efforts with ``delta`` on ``load_joint`` from reading ``start`` after the baseline."""

    def at(tick):
        values = [1.0] * len(NAMES)
        reading = tick - BASELINE_TICKS
        if reading >= start and (end is None or reading < end):
            values[NAMES.index(load_joint)] += delta
        return values

    return at


@pytest.fixture
def payload_arm(monkeypatch):
    def build(effort_fn, joint_state=True, estop=False):
        clock = _Clock(effort_fn)
        monkeypatch.setattr(arm_module, "time", clock)
        arm = RobotArm.__new__(RobotArm)
        arm._log = FakeLogger()
        arm._estop = estop
        arm._goal_handle = None
        arm._latest_joint_state = (
            SimpleNamespace(name=list(NAMES), effort=[1.0] * len(NAMES))
            if joint_state
            else None
        )
        clock.arm = arm
        return arm

    return build


def wait(arm, **kwargs):
    return arm.wait_for_payload(1.5, 2.0, ["joint2", "joint3", "joint5"], **kwargs)


def test_a_sustained_load_is_a_payload(payload_arm):
    assert wait(payload_arm(efforts()))


def test_a_brief_tug_is_not_a_payload(payload_arm):
    assert not wait(payload_arm(efforts(start=5, end=12)))


def test_a_small_load_is_not_a_payload(payload_arm):
    assert not wait(payload_arm(efforts(delta=1.0)))


def test_load_on_an_unlisted_joint_is_ignored(payload_arm):
    assert not wait(payload_arm(efforts(load_joint="joint1")))


def test_no_joint_states_is_no_payload(payload_arm):
    assert not wait(payload_arm(efforts(), joint_state=False))


def test_an_estop_while_waiting_aborts(payload_arm):
    arm = payload_arm(efforts(delta=0.0))
    arm._estop = True
    with pytest.raises(PickAborted):
        wait(arm)


# ============================================================================
# manipulation_core
# ============================================================================


class _GoalHandle:
    def __init__(self, request, cancel=False):
        self.request = request
        self.is_cancel_requested = cancel
        self.state = None

    def succeed(self):
        self.state = "succeeded"

    def abort(self):
        self.state = "aborted"

    def canceled(self):
        self.state = "canceled"


@pytest.fixture
def core():
    node = ManipulationCore.__new__(ManipulationCore)
    node.arm = make_arm()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._follow_lock = threading.Lock()
    node._task_lock = threading.Lock()
    node._face = follow_pipeline.FaceState()
    node._person = follow_pipeline.PersonState()
    return node


def serve(core, goal_handle):
    assert core.goal_callback(goal_handle.request) == GoalResponse.ACCEPT
    return asyncio.run(core.take_from_hand_callback(goal_handle))


def request():
    return SimpleNamespace(
        hand_point=wrist_point(), grasp_below_hand=0.0, release_timeout=0.0
    )


def test_core_reports_a_grip_without_payload_as_success(core):
    core.arm = make_arm(payload_results=[False])
    goal = _GoalHandle(request())
    result = serve(core, goal)

    assert goal.state == "succeeded"
    assert (result.success, result.payload_detected) == (True, False)
    assert core.arm.calls[0] == "bind_goal" and core.arm.calls[-1] == "release_goal"
    assert not core._task_lock.locked()


def test_core_aborts_a_failed_take(core):
    core.arm = make_arm(tf_fails=True)
    point_request = request()
    point_request.hand_point = wrist_point(frame="camera")
    goal = _GoalHandle(point_request)
    result = serve(core, goal)

    assert goal.state == "aborted" and not result.success
    assert not core._task_lock.locked()


@pytest.mark.parametrize("cancel, state", [(True, "canceled"), (False, "aborted")])
def test_core_maps_an_abort_to_canceled_or_aborted(core, cancel, state):
    core.arm = make_arm(abort_after=0)
    goal = _GoalHandle(request(), cancel=cancel)
    serve(core, goal)

    assert goal.state == state
    assert core.arm.calls[-1] == "release_goal"
    assert not core._task_lock.locked()


def test_core_refuses_under_estop_without_moving(core):
    core.arm = make_arm(estop=True)
    goal = _GoalHandle(request())
    serve(core, goal)

    assert goal.state == "aborted"
    assert not MOTION & set(core.arm.calls)
    assert not core._task_lock.locked()
