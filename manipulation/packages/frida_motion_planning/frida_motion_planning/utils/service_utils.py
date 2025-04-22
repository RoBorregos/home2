from frida_constants.xarm_configurations import XARM_CONFIGURATIONS
from frida_constants.manipulation_constants import DEG2RAD, RAD2DEG
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import GetJoints
from typing import List, Union


def move_joint_positions(
    move_joints_action_client,
    joint_positions: Union[List[float], dict] = None,
    named_position: str = None,
    velocity: float = 0.1,
    degrees=False,  # set to true if joint_positions are in degrees
):
    """Set position of joints.
    If joint_positions is a dict, keys are treated as joint_names
    and values as joint positions.
    Named position has priority over joint_positions.
    """

    # let the server pick the default values
    def _send_joint_goal(
        move_joints_action_client,
        joint_names=[],
        joint_positions=[],
        velocity=0.0,
        acceleration=0.0,
        planner_id="",
    ):
        # print(" QUE PEDOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        # time.sleep(5)
        goal_msg = MoveJoints.Goal()
        # print("MoveJoints goal message created")
        # time.sleep(5)
        goal_msg.joint_names = joint_names
        # print("Joint names set")
        # time.sleep(5)
        goal_msg.joint_positions = joint_positions
        # print("Joint positions set")
        # time.sleep(5)
        goal_msg.velocity = velocity
        # print("Velocity set")
        # time.sleep(5)
        goal_msg.acceleration = acceleration
        # print("Acceleration set")
        # time.sleep(5)
        goal_msg.planner_id = planner_id
        # print(" QUE PEDIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
        # time.sleep(5)
        move_joints_action_client.wait_for_server()

        return move_joints_action_client.send_goal_async(goal_msg)

    if named_position:
        joint_positions = XARM_CONFIGURATIONS[named_position]["joints"]
        degrees = XARM_CONFIGURATIONS[named_position]["degrees"]

    # Determine format of joint_positions and apply degree conversion if needed.
    if isinstance(joint_positions, dict):
        joint_names = list(joint_positions.keys())
        joint_vals = list(joint_positions.values())
        if degrees:
            joint_vals = [x * DEG2RAD for x in joint_vals]
    elif isinstance(joint_positions, list):
        joint_names = []
        joint_vals = joint_positions.copy()
        if degrees:
            joint_vals = [x * DEG2RAD for x in joint_vals]
    else:
        return False

    future = _send_joint_goal(
        move_joints_action_client=move_joints_action_client,
        joint_names=joint_names,
        joint_positions=joint_vals,
        velocity=velocity,
    )
    # Check result
    future = wait_for_future(future)
    result = future.result().get_result().result
    return result.success


# @service_check("get_joints_positions", -1, TIMEOUT)
def get_joint_positions(
    get_joints_client,
    degrees=False,  # set to true to return in degrees
) -> dict:
    """Get the current joint positions"""
    get_joints_client.wait_for_service(timeout_sec=3)
    future = get_joints_client.call_async(GetJoints.Request())
    future = wait_for_future(future)
    result = future.result()
    if degrees:
        result.joint_positions = [x * RAD2DEG for x in result.joint_positions]
    return dict(zip(result.joint_names, result.joint_positions))
