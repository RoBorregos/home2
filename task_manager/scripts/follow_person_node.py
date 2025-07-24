#!/usr/bin/env python3

"""
Follow person node
"""

from time import time as t

import rclpy
from frida_interfaces.srv import FollowFace
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from utils.logger import Logger
from xarm_msgs.srv import MoveVelocity, SetInt16, GetFloat32List
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from frida_pymoveit2.robots import xarm6
from frida_constants.manipulation_constants import (
    FOLLOW_FACE_SPEED,
)


XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_JOINT_ANGLE_SERVICE = "/xarm/get_servo_angle"
DASHGO_CMD_VEL = "/cmd_vel"

# class subtask_manager:
#     vision: VisionTasks
#     manipulation: ManipulationTasks

TIMEOUT = 5.0
MAX_ERROR = 0.2
MAX_ROTATIONAL_VEL = 1.0
CENTROID_TOIC = "/vision/tracker_centroid"

SPEED = 1.0


class FollowPersonNode(Node):
    """Class to manage demo tasks"""

    TASK_STATES = {"START": 0, "FOLLOW_FACE": 3}

    Multiplier = 5

    def __init__(self):
        """Initialize the node"""
        super().__init__("follow_person_node_started")
        callback_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/xarm6_traj_controller/joint_trajectory", qos_profile
        )
        self.joint_states_topic = "/joint_states"
        # create listener
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            qos_profile,
            callback_group=callback_group,
        )

        self.follow_face_sub = self.create_subscription(
            Point, CENTROID_TOIC, self.follow_callback, 10
        )
        self.dashgo_cmd_vel_sub = self.create_subscription(
            PoseStamped,
            DASHGO_CMD_VEL,
            self.get_dashgo_cmd_vel_callback,
            qos_profile,
            callback_group=callback_group,
        )
        self.dashgo_cmd_vel = 0.0
        self.current_x = 0
        self.current_y = 0
        self.max_x = 20
        self.min_x = -20
        self.max_y = 10
        self.min_y = -10
        self.max_delta_x = 2
        self.max_delta_y = 1
        self.min_delta_x = -2
        self.min_delta_y = -1
        self.prevx = 0.0
        self.prevy = 0.0
        self.counter_move = t()
        self.dashg_cmd_vel = 0.0
        self.angle_status = None

        self.x_delta_multiplier = self.Multiplier
        self.y_delta_multiplier = self.Multiplier / 2

        self.get_logger().info("Follow person node has started.")

        self.mode_client = self.create_client(SetInt16, XARM_SETMODE_SERVICE)

        self.joint_angle_client = self.create_client(GetFloat32List, XARM_JOINT_ANGLE_SERVICE)

        self.state_client = self.create_client(SetInt16, XARM_SETSTATE_SERVICE)

        self.move_client = self.create_client(MoveVelocity, XARM_MOVEVELOCITY_SERVICE)

        if not self.move_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "Move client not initialized")

        self.service = self.create_service(
            FollowFace, "/follow_person", self.follow_person_callback
        )

        self.is_following_person = False

        self.is_following_person = False

        self.follow_face = {"x": 0, "y": 0}

        self.flag_active_face = False
        self.deactivate = False

        self.deactivate = False

        self.last_follow_msg = 0

        self.create_timer(
            0.1,
            self.run,
            callback_group=callback_group,
        )

    def follow_callback(self, msg: Point):
        """Callback for the face following subscriber"""
        self.last_follow_msg = time.time()
        self.follow_face["x"] = msg.x
        self.follow_face["y"] = msg.y
        self.flag_active_face = True

    def joint_states_callback(self, msg: JointState):
        relevant_indices = []
        for i, joint_name in enumerate(msg.name):
            if joint_name in xarm6.joint_names():
                relevant_indices.append(i)
        # Filtra solo los joints del brazo
        if len(relevant_indices) == 0:
            return
        self.joint_states = JointState()
        self.joint_states.name = [msg.name[i] for i in relevant_indices]
        self.joint_states.position = [msg.position[i] for i in relevant_indices]

    def get_dashgo_cmd_vel_callback(self, msg: PoseStamped):
        """Callback to get the dashgo rotational velocity"""
        self.dashgo_cmd_vel = msg.pose.orientation.z

    def follow_person_callback(self, request: FollowFace.Request, response: FollowFace.Response):
        self.is_following_person = request.follow_face
        if not self.is_following_person:
            self.move_to(0.0, 0.0)
        response.success = True
        return response

    def move_to(self, x: float, y: float):
        Logger.info(self, f"x: {x} , y: {y}")
        # Prepare velocity command for joint trajectory controller
        traj_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        # Set motion
        x = x * -1
        if x > 0.1:
            x_vel = 0.1
        elif x < -0.1:
            x_vel = -0.1
        else:
            x_vel = x
        if y > 0.1:
            y_vel = 0.1
        elif y < -0.1:
            y_vel = -0.1
        else:
            y_vel = y
        speed_multiplier = FOLLOW_FACE_SPEED * SPEED
        x_vel *= speed_multiplier
        y_vel *= speed_multiplier

        for name, position in zip(self.joint_states.name, self.joint_states.position):
            if name == "joint1":
                if position < -2.5 or position > -0.5:
                    point.positions.append(position)
                    continue
                point.positions.append(position + x_vel)
                traj_msg.joint_names.append(name)
            elif name == "joint5":
                traj_msg.joint_names.append(name)
                point.positions.append(position + y_vel)
            else:
                traj_msg.joint_names.append(name)
                point.positions.append(position)

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e8)  # 0.2s
        traj_msg.points.append(point)
        try:
            # Publish to joint trajectory controller
            self.traj_pub.publish(traj_msg)
        except Exception as e:
            Logger.error(self, f"Error sending joint trajectory velocity: {e}")
            return 123
        return 123

    def go_zero(self):
        # Prepare velocity command for joint trajectory controller
        self.get_logger().info("GOZEROOO")
        traj_msg = JointTrajectory()
        point = JointTrajectoryPoint()

        for name, position in zip(self.joint_states.name, self.joint_states.position):
            if name == "joint1":
                point.positions.append(-1.57)
                traj_msg.joint_names.append(name)
            elif name == "joint5":
                traj_msg.joint_names.append(name)
            else:
                traj_msg.joint_names.append(name)
                point.positions.append(position)

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(5e8)  # 0.2s
        traj_msg.points.append(point)

        self.get_logger().info(f"traj: {traj_msg}")
        try:
            # Publish to joint trajectory controller
            self.traj_pub.publish(traj_msg)
        except Exception as e:
            Logger.error(self, f"Error sending joint trajectory velocity: {e}")
            return 123
        return 123

    def state_response_callback(self, future):
        """Callback for state service response"""
        try:
            result = future.result()
            if result:
                Logger.info(self, "Arm moved")
            else:
                Logger.error(self, "Failed to move arm")
        except Exception as e:
            Logger.error(self, f"move service call failed: {str(e)}")

    def update_angle(self, future):
        try:
            result = future.result()
            if result:
                self.angle_status = result.datas
                # print(self.angle_status)
            else:
                Logger.error(self, "Failed to get joint data")
        except Exception as e:
            Logger.error(self, f"Failed to get joint data: {str(e)}")

    def get_joint_angle(self):
        try:
            angle_msg = GetFloat32List.Request()
            future_move = self.joint_angle_client.call_async(angle_msg)
            future_move.add_done_callback(self.update_angle)  # Fire-and-forget
        except Exception as e:
            Logger.error(self, f"Error getting joints arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

    def get_follow_face(self):
        """Get the face to follow"""
        if self.flag_active_face:
            self.flag_active_face = False
            return self.follow_face["x"], self.follow_face["y"]
        else:
            return None, None

    def run(self):
        if not self.is_following_person:
            return

        self.get_joint_angle()

        x, y = self.get_follow_face()

        if (time.time() - self.last_follow_msg) > 1.0:
            self.go_zero()
            return
        if x is not None:
            if self.angle_status is not None:
                vel = self.error_to_velocity(x, y)
                if self.angle_status[0] > -2.26893 and self.angle_status[0] < -0.872665:
                    self.move_to(vel, 0.0)
                else:
                    if self.angle_status[0] <= -2.26893 and vel < 0:
                        self.move_to(vel, 0.0)
                    elif self.angle_status[0] >= -0.872665 and vel > 0:
                        self.move_to(vel, 0.0)
                    else:
                        self.move_to(0.0, 0.0)

    def error_to_velocity(self, x: float, y: float):
        """Convert error to velocity"""
        KP = 0.7
        x_vel = KP * x
        x_vel = max(min(x_vel, MAX_ROTATIONAL_VEL), -MAX_ROTATIONAL_VEL)

        x_vel_sign = np.sign(x_vel)
        dashgo_vel_sign = np.sign(self.dashgo_cmd_vel)

        if x_vel_sign == dashgo_vel_sign and abs(x) <= MAX_ERROR:
            return 0.0

        return x_vel


def main(args=None):
    """Main function"""

    rclpy.init(args=args)

    demo_task_manager = FollowPersonNode()

    rclpy.spin(demo_task_manager)

    demo_task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
