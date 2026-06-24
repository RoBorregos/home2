import copy
from geometry_msgs.msg import PointStamped, PoseStamped
from frida_interfaces.srv import GraspDetection
from frida_constants.manipulation_constants import PICK_VELOCITY
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.service_utils import move_joint_positions
from frida_pymoveit2.robots.xarm6 import joint_names as xarm6_joint_names
from pick_and_place.utils.self_collision_utils import compute_ik


def get_grasps(grasp_detection_client, object_cloud, cgf_path: str):
    # wait for the service to be available
    if not grasp_detection_client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("Service not available")
    request = GraspDetection.Request()
    request.input_cloud = object_cloud
    request.cfg_path = cgf_path
    future = grasp_detection_client.call_async(request)
    future = wait_for_future(future, timeout=10)

    if not future:
        return [], []

    response = future.result()

    return response.grasp_poses, response.grasp_scores


def fake_grasps(object_point: PointStamped):
    grasp_pose1 = PoseStamped()
    grasp_pose1.header.frame_id = object_point.header.frame_id
    grasp_pose1.pose.position.x = object_point.point.x
    grasp_pose1.pose.position.y = object_point.point.y
    grasp_pose1.pose.position.z = object_point.point.z + 0.10
    grasp_pose1.pose.orientation.x = 1.0
    grasp_pose1.pose.orientation.y = 0.0
    grasp_pose1.pose.orientation.z = 0.0
    grasp_pose1.pose.orientation.w = 0.0

    grasp_pose2 = copy.deepcopy(grasp_pose1)
    grasp_pose2.pose.position.z = object_point.point.z + 0.25

    grasp_poses = [grasp_pose1, grasp_pose2]
    grasp_scores = [0.9, 0.8]

    return grasp_poses, grasp_scores


def move_to_pregrasp_nearest_ik(
    compute_ik_client,
    move_joints_action_client,
    pose_stamped,
    latest_joint_state=None,
    velocity: float = PICK_VELOCITY,
    fallback_move_to_pose=None,
    logger=None,
) -> bool:
    """
    Move to a pre-grasp pose planning in joint space toward the IK solution
    nearest to the current configuration.

    A free pose goal lets the planner pick any IK solution of the goal region,
    which can wind joint1 (the base) almost 360 deg "around the back". Seeding
    compute_ik with the current joint state returns the closest collision-free
    config, and a joint goal to it keeps the motion short.

    Args:
        compute_ik_client: client for /compute_ik (GetPositionIK).
        move_joints_action_client: action client for the joint-goal move.
        pose_stamped: target pose of GRASP_LINK_FRAME.
        latest_joint_state: optional JointState used as the IK seed.
        velocity: motion velocity scaling.
        fallback_move_to_pose: optional callable(pose, velocity=...) -> (handle,
            result) used when IK is unavailable/fails. Its result is expected to
            expose `.result.success`.
        logger: optional rclpy logger for diagnostics.

    Returns:
        True if the pre-grasp was reached.
    """

    def _info(msg):
        if logger is not None:
            logger.info(msg)

    def _warn(msg):
        if logger is not None:
            logger.warn(msg)

    ik_resp = compute_ik(
        compute_ik_client,
        pose_stamped,
        avoid_collisions=True,
        seed_joint_state=latest_joint_state,
    )
    if ik_resp is not None and ik_resp.error_code.val == ik_resp.error_code.SUCCESS:
        sol = dict(
            zip(
                ik_resp.solution.joint_state.name,
                ik_resp.solution.joint_state.position,
            )
        )
        arm_names = xarm6_joint_names()
        if all(n in sol for n in arm_names):
            # Pass the named-joint dict
            joints = {n: float(sol[n]) for n in arm_names}
            _info("[PreGrasp] Moving via nearest-IK joint goal")
            return bool(
                move_joint_positions(
                    move_joints_action_client,
                    joint_positions={"joints": joints, "degrees": False},
                    velocity=velocity,
                    wait=True,
                )
            )

    _warn("[PreGrasp] Nearest-IK unavailable, falling back to pose goal")
    if fallback_move_to_pose is None:
        return False
    _, pre_result = fallback_move_to_pose(pose_stamped, velocity=velocity)
    return bool(pre_result.result.success)
