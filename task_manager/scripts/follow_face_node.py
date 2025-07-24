#!/usr/bin/env python3

"""
Task Manager for Demo follow face
"""

from time import time as t

import rclpy
from frida_interfaces.srv import FollowFace
from geometry_msgs.msg import Point
from rclpy.node import Node
from utils.logger import Logger
from xarm_msgs.srv import MoveVelocity, SetInt16
from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    MOVEIT_MODE,
    FOLLOW_FACE_TOLERANCE,
)
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from frida_motion_planning.utils.ros_utils import wait_for_future
from std_srvs.srv import Empty


XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"

# class subtask_manager:
#     vision: VisionTasks
#     manipulation: ManipulationTasks

TIMEOUT = 5.0


class FollowFaceNode(Node):
    """Class to manage demo tasks"""

    TASK_STATES = {"START": 0, "FOLLOW_FACE": 3}

    Multiplier = 5

    def __init__(self):
        """Initialize the node"""
        super().__init__("follow_face_node")
        callback_group = ReentrantCallbackGroup()
        # self.subtask_manager = subtask_manager()
        # self.subtask_manager.vision = VisionTasks(
        #     self, task="DEMO", mock_data=False)
        # self.subtask_manager.manipulation = ManipulationTasks(
        #     self, task="DEMO", mock_data=False)
        self.follow_face_sub = self.create_subscription(
            Point, "/vision/follow_face", self.follow_callback, 2, callback_group=callback_group
        )
        # self.subtask_manager.hri = HRITasks(self, config=test_hri_config)

        # change

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

        self.x_delta_multiplier = self.Multiplier
        self.y_delta_multiplier = self.Multiplier / 2

        self.get_logger().info("DemoTaskManager has started.")

        self.mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=callback_group
        )

        self.state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=callback_group
        )

        self.move_client = self.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE, callback_group=callback_group
        )

        self.reset_controller_client = self.create_client(
            Empty, "/manipulation/reset_xarm_controller", callback_group=callback_group
        )

        if not self.move_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "Move client not initialized")

        if not self.state_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "AAAA1")

        if not self.mode_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "AAAA3")

        self.service = self.create_service(
            FollowFace,
            "/follow_face",
            self.follow_face_callback,
            callback_group=callback_group,
        )

        self.is_following_face_active = False
        self.arm_moving = False

        self.follow_face = {"x": 0, "y": 0}
        self.last_face_detection_time = 0

        self.flag_active_face = False

        self.create_timer(0.1, self.run, callback_group=callback_group)

    def set_state_speed(self):
        Logger.info(self, "Activating arm for speed calls")

        # Set state
        state_request = SetInt16.Request()
        state_request.data = 0
        # Set mode
        # 4: velocity control mode
        mode_request = SetInt16.Request()
        mode_request.data = 4

        tries = 2
        for i in range(tries):
            try:
                Logger.info(self, "Setting mode")
                future_mode = self.mode_client.call_async(mode_request)
                future_mode = wait_for_future(future_mode)
                if not future_mode:
                    Logger.error(self, "Failed to set mode")
                    continue
                else:
                    Logger.success(self, "Mode set")
                Logger.info(self, "Setting state")
                future_state = self.state_client.call_async(state_request)
                future_state = wait_for_future(future_state)
                if not future_state:
                    Logger.error(self, "Failed to set state")
                else:
                    Logger.success(self, "State set")
                    break
            except Exception as e:
                Logger.error(self, f"Error Activating arm: {e}")
        time.sleep(2)

    def set_state_moveit(self):
        Logger.info(self, "Activating arm for moveit")

        # Set state
        state_request = SetInt16.Request()
        state_request.data = 0
        # Set mode
        mode_request = SetInt16.Request()
        mode_request.data = MOVEIT_MODE

        tries = 2
        for i in range(tries):
            try:
                # Logger.info(self, "Resetting controller")
                # future_controller = self.reset_controller_client.call_async(Empty.Request())
                # future_controller = wait_for_future(future_controller)
                # if not future_controller:
                #     Logger.error("Failed to reset controller")
                # else:
                #     Logger.success(self, "Success in resetting controller")
                Logger.info(self, "Setting mode")
                future_mode = self.mode_client.call_async(mode_request)
                future_mode = wait_for_future(future_mode)
                if not future_mode:
                    Logger.error(self, "Failed to set mode")
                    continue
                else:
                    Logger.success(self, "Mode set")
                Logger.info(self, "Setting state")
                future_state = self.state_client.call_async(state_request)
                future_state = wait_for_future(future_state)
                if not future_state:
                    Logger.error(self, "Failed to set state")
                else:
                    Logger.success(self, "State set")
                    break
            except Exception as e:
                Logger.error(self, f"Error Activating arm: {e}")
        time.sleep(2)

    def follow_callback(self, msg: Point):
        """Callback for the face following subscriber"""
        self.follow_face["x"] = msg.x
        self.follow_face["y"] = msg.y
        self.last_face_detection_time = time.time()
        self.flag_active_face = True

    def get_follow_face(self):
        """Get the face to follow"""
        if self.flag_active_face:
            self.flag_active_face = False
            if time.time() - self.last_face_detection_time > FACE_RECOGNITION_LIFETIME:
                Logger.warn(self, "OVER LIFETIME")
                return None, None
            return self.follow_face["x"], self.follow_face["y"]
        else:
            return None, None

    def follow_face_callback(self, request: FollowFace.Request, response: FollowFace.Response):
        self.is_following_face_active = request.follow_face
        time.sleep(1)
        if not self.is_following_face_active:
            while self.arm_moving:
                Logger.info(self, "Waiting for arm to stop moving")
                time.sleep(0.1)
            self.move_to(0.0, 0.0)
            time.sleep(2)

            self.set_state_moveit()
            time.sleep(2)
        else:
            self.set_state_speed()
            time.sleep(2)
        response.success = True
        return response

    def move_to(self, x: float, y: float):
        if self.arm_moving:
            return
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

        speed_multiplier = FOLLOW_FACE_SPEED
        x_vel *= speed_multiplier
        y_vel *= speed_multiplier

        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = [x_vel, 0.0, 0.0, 0.0, y_vel, 0.0, 0.0]

        try:
            self.arm_moving = True
            future_move = self.move_client.call_async(motion_msg)
            future_move.add_done_callback(self.state_response_callback)  # Fire-and-forget

        except Exception as e:
            Logger.error(self, f"Error desactivating arm: {e}")
            # return self.STATE["EXECUTION_ERROR"]
            return 123

        return 123

    def state_response_callback(self, future):
        """Callback for state service response"""
        try:
            result = future.result()
            if not result:
                Logger.error(self, "Failed to move arm")
        except Exception as e:
            Logger.error(self, f"move service call failed: {str(e)}")
        finally:
            self.arm_moving = False

    def run(self):
        if not self.is_following_face_active:
            return
        """Running main loop"""
        # Follow face task
        x, y = self.get_follow_face()
        # x *= -1
        # y*=-1
        if x is None and y is None:
            if self.prevx != 0.0 and self.prevy != 0.0 and t() - self.counter_move >= 1:
                self.move_to(0.0, 0.0)
            else:
                pass
            # pass
        else:
            y *= -1
            if abs(x) > FOLLOW_FACE_TOLERANCE or abs(y) > FOLLOW_FACE_TOLERANCE:
                Logger.info(self, f"Moving X: {x} Y: {y}")
                self.move_to(x, y)
                self.prevx = x
                self.prevy = y
                self.counter_move = t()
            else:
                self.move_to(0.0, 0.0)
                self.prevx = x
                self.prevy = y


def main(args=None):
    """Main function"""

    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    follow_face_node = FollowFaceNode()
    executor.add_node(follow_face_node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
