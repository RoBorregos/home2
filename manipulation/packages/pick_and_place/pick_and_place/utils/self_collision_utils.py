import copy

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import ContactInformation
from frida_constants.manipulation_constants import GRASP_LINK_FRAME
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_pymoveit2.robots.xarm6 import MOVE_GROUP_ARM

IK_TIMEOUT_SEC = 0.2
APPROACH_STEP = 0.01
MAX_JOINT_JUMP = 0.5
CAMERA_LINK = "zed_camera_link"


def compute_ik(
    compute_ik_client,
    pose_stamped,
    avoid_collisions: bool,
    seed_joint_state=None,
):
    """
    Solve inverse kinematics for `pose_stamped`

    Args:
        compute_ik_client: client for /compute_ik (GetPositionIK).
        pose_stamped: target pose of GRASP_LINK_FRAME.
        avoid_collisions: whether the IK solution must be collision-free.
        seed_joint_state: optional JointState used as the IK seed.

    Returns:
        The GetPositionIK response (inspect `.error_code` / `.solution`), or
        None if the service is unavailable or the call failed.
    """
    if not compute_ik_client.wait_for_service(timeout_sec=2.0):
        return None

    req = GetPositionIK.Request()
    req.ik_request.group_name = MOVE_GROUP_ARM
    req.ik_request.ik_link_name = GRASP_LINK_FRAME
    req.ik_request.pose_stamped = pose_stamped
    req.ik_request.avoid_collisions = avoid_collisions
    req.ik_request.timeout.sec = 0
    req.ik_request.timeout.nanosec = int(IK_TIMEOUT_SEC * 1e9)
    if seed_joint_state is not None:
        req.ik_request.robot_state.joint_state = seed_joint_state

    future = compute_ik_client.call_async(req)
    wait_for_future(future)
    return future.result()


def endpoint_self_collides(
    compute_ik_client,
    state_validity_client,
    pose_stamped: PoseStamped,
    latest_joint_state=None,
    logger=None,
) -> bool:
    """
    Check whether the robot would be in self-collision at the given pose.

    Uses MoveIt's /compute_ik followed by /check_state_validity, filtering the
    reported contacts to ROBOT_LINK <-> ROBOT_LINK pairs only. Collisions with
    WORLD_OBJECT (clothes/basket) are intentionally ignored.

    Args:
        compute_ik_client: client for /compute_ik (GetPositionIK).
        state_validity_client: client for /check_state_validity (GetStateValidity).
        pose_stamped: target pose of GRASP_LINK_FRAME to validate.
        latest_joint_state: optional JointState used as the IK seed.
        logger: optional rclpy logger for diagnostics.

    Returns:
        True if the pose should be skipped (self-collision or unreachable),
        False if it is safe to attempt. Fails open (False) if the MoveIt
        services are unavailable.
    """

    def _warn(msg):
        if logger is not None:
            logger.warn(msg)

    ik_resp = compute_ik(
        compute_ik_client,
        pose_stamped,
        avoid_collisions=False,
        seed_joint_state=latest_joint_state,
    )
    if ik_resp is None:
        _warn("[SelfCollision] /compute_ik unavailable, skipping check")
        return False
    if ik_resp.error_code.val != ik_resp.error_code.SUCCESS:
        _warn(
            f"[SelfCollision] IK failed (error_code={ik_resp.error_code.val}); "
            "treating endpoint as unreachable"
        )
        return True

    if not state_validity_client.wait_for_service(timeout_sec=2.0):
        _warn("[SelfCollision] /check_state_validity unavailable, skipping check")
        return False

    sv_req = GetStateValidity.Request()
    sv_req.robot_state = ik_resp.solution
    sv_req.group_name = MOVE_GROUP_ARM

    sv_future = state_validity_client.call_async(sv_req)
    wait_for_future(sv_future)
    sv_resp = sv_future.result()
    if sv_resp is None:
        _warn("[SelfCollision] state validity call returned None, skipping check")
        return False

    if sv_resp.valid:
        return False

    # Not valid: only treat robot-robot contacts as a blocking self-collision.
    for c in sv_resp.contacts:
        if (
            c.body_type_1 == ContactInformation.ROBOT_LINK
            and c.body_type_2 == ContactInformation.ROBOT_LINK
        ):
            _warn(
                f"[SelfCollision] Robot self-collision between "
                f"'{c.contact_body_1}' and '{c.contact_body_2}'"
            )
            return True

    # Invalid only due to world/attached objects (e.g. clothes) -> allowed.
    return False


def approach_collides(
    compute_ik_client,
    state_validity_client,
    pose_stamped: PoseStamped,
    direction,
    distance: float,
    latest_joint_state=None,
    logger=None,
    self_collisions: bool = True,
) -> bool:
    """True if the straight drive from `pose_stamped` (IK + state validity every APPROACH_STEP) hits the camera-vs-world or, with `self_collisions`, arm-vs-arm; fails closed if MoveIt is down."""

    def _warn(msg):
        if logger is not None:
            logger.warn(f"[ApproachCheck] {msg}")

    if not state_validity_client.wait_for_service(timeout_sec=2.0):
        _warn("/check_state_validity unavailable, cannot check the approach")
        return True

    steps = int(np.ceil(distance / APPROACH_STEP - 1e-6))
    seed, previous = latest_joint_state, None
    for step in range(steps + 1):
        at = f"step {step}/{steps}"
        travelled = min(step * APPROACH_STEP, distance)
        pose = copy.deepcopy(pose_stamped)
        pose.pose.position.x += float(direction[0]) * travelled
        pose.pose.position.y += float(direction[1]) * travelled
        pose.pose.position.z += float(direction[2]) * travelled

        ik_resp = compute_ik(
            compute_ik_client, pose, avoid_collisions=False, seed_joint_state=seed
        )
        if ik_resp is None:
            _warn("/compute_ik unavailable, cannot check the approach")
            return True
        if ik_resp.error_code.val != ik_resp.error_code.SUCCESS:
            _warn(f"IK failed at {at} (error_code={ik_resp.error_code.val})")
            return True

        seed = ik_resp.solution.joint_state
        joints = np.array(seed.position)
        if (
            self_collisions
            and previous is not None
            and np.abs(joints - previous).max() > MAX_JOINT_JUMP
        ):
            _warn(f"joints jump at {at}; the arm cannot follow the drive")
            return True
        previous = joints

        sv_req = GetStateValidity.Request()
        sv_req.robot_state = ik_resp.solution
        sv_req.group_name = MOVE_GROUP_ARM
        sv_future = state_validity_client.call_async(sv_req)
        wait_for_future(sv_future)
        sv_resp = sv_future.result()
        if sv_resp is None:
            _warn("state validity call returned None, cannot check the approach")
            return True

        for c in sv_resp.contacts:
            bodies = (c.contact_body_1, c.contact_body_2)
            robot_pair = (
                c.body_type_1 == ContactInformation.ROBOT_LINK
                and c.body_type_2 == ContactInformation.ROBOT_LINK
            )
            if (robot_pair and self_collisions) or (
                CAMERA_LINK in bodies and not robot_pair
            ):
                _warn(f"'{bodies[0]}' hits '{bodies[1]}' at {at}")
                return True

    return False
