from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import ContactInformation
from frida_constants.manipulation_constants import GRASP_LINK_FRAME
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_pymoveit2.robots.xarm6 import MOVE_GROUP_ARM

IK_TIMEOUT_SEC = 0.2


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
    WORLD_OBJECT (clothes/basket) are intentionally ignored, so callers that
    legitimately push into the environment (e.g. peak picks into laundry) are
    only blocked when the arm would hit itself.

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

    if not compute_ik_client.wait_for_service(timeout_sec=2.0):
        _warn("[SelfCollision] /compute_ik unavailable, skipping check")
        return False

    ik_req = GetPositionIK.Request()
    ik_req.ik_request.group_name = MOVE_GROUP_ARM
    ik_req.ik_request.ik_link_name = GRASP_LINK_FRAME
    ik_req.ik_request.pose_stamped = pose_stamped
    ik_req.ik_request.avoid_collisions = False
    ik_req.ik_request.timeout.sec = 0
    ik_req.ik_request.timeout.nanosec = int(IK_TIMEOUT_SEC * 1e9)
    if latest_joint_state is not None:
        ik_req.ik_request.robot_state.joint_state = latest_joint_state

    ik_future = compute_ik_client.call_async(ik_req)
    wait_for_future(ik_future)
    ik_resp = ik_future.result()
    if ik_resp is None or ik_resp.error_code.val != ik_resp.error_code.SUCCESS:
        code = None if ik_resp is None else ik_resp.error_code.val
        _warn(
            f"[SelfCollision] IK failed (error_code={code}); treating endpoint "
            "as unreachable"
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
