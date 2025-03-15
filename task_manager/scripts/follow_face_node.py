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
        super().__init__("demo_task_manager")
        # self.subtask_manager = subtask_manager()
        # self.subtask_manager.vision = VisionTasks(
        #     self, task="DEMO", mock_data=False)
        # self.subtask_manager.manipulation = ManipulationTasks(
        #     self, task="DEMO", mock_data=False)
        self.follow_face_sub = self.create_subscription(
            Point, "/vision/follow_face", self.follow_callback, 10
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

        self.mode_client = self.create_client(SetInt16, XARM_SETMODE_SERVICE)

        self.state_client = self.create_client(SetInt16, XARM_SETSTATE_SERVICE)

        self.move_client = self.create_client(MoveVelocity, XARM_MOVEVELOCITY_SERVICE)

        if not self.move_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "Move client not initialized")

        if not self.state_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "AAAA1")

        if not self.mode_client.wait_for_service(timeout_sec=TIMEOUT):
            Logger.warn(self, "AAAA3")

        self.service = self.create_service(FollowFace, "/follow_face", self.follow_face_callback)

        self.is_following_face_active = False

        self.follow_face = {"x": 0, "y": 0}

        self.flag_active_face = False

        self.create_timer(0.1, self.run)

    def set_state(self):
        Logger.info(self, "Activating arm")

        # Set state
        state_request = SetInt16.Request()
        state_request.data = 0
        # Set mode
        # 0: position control mode
        mode_request = SetInt16.Request()
        mode_request.data = 0

        try:
            future_mode = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self, future_mode, timeout_sec=TIMEOUT)

            future_state = self.state_client.call_async(state_request)
            rclpy.spin_until_future_complete(self, future_state, timeout_sec=TIMEOUT)
        except Exception as e:
            Logger.error(self, f"Error Activating arm: {e}")

    def follow_callback(self, msg: Point):
        """Callback for the face following subscriber"""
        self.follow_face["x"] = msg.x
        self.follow_face["y"] = msg.y
        self.flag_active_face = True

    def get_follow_face(self):
        """Get the face to follow"""
        if self.flag_active_face:
            self.flag_active_face = False
            return self.follow_face["x"], self.follow_face["y"]
        else:
            return None, None

    def follow_face_callback(self, request: FollowFace.Request, response: FollowFace.Response):
        self.is_following_face_active = request.follow_face
        if not self.is_following_face_active:
            self.move_to(0.0, 0.0)
        else:
            self.set_state()
        response.success = True
        return response

    def move_to(self, x: float, y: float):
        Logger.info(self, "Moving arm with velocity")

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

        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = [x_vel, 0.0, 0.0, 0.0, y_vel, 0.0, 0.0]

        try:
            print(f"mock moving to {x} {y}")
            future_move = self.move_client.call_async(motion_msg)
            future_move.add_done_callback(self.state_response_callback)  # Fire-and-forget

        except Exception as e:
            Logger.error(self, f"Error desactivating arm: {e}")
            # return self.STATE["EXECUTION_ERROR"]
            return 123

        Logger.success(self, "Arm moved")
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

    def run(self):
        if not self.is_following_face_active:
            return
        """Running main loop"""
        # Follow face task
        Logger.state(self, "Follow face task")
        x, y = self.get_follow_face()
        # x *= -1
        # y*=-1
        if x is None and y is None:
            Logger.info(
                self,
                f"prev = {self.prevx} t = {t()} counter = {self.counter_move} resta = {t() - self.counter_move}",
            )
            if self.prevx != 0.0 and self.prevy != 0.0 and t() - self.counter_move >= 1:
                self.move_to(0.0, 0.0)
            else:
                pass
            # pass
        else:
            y *= -1
            print(f"x and y {x} {y}")
            if x > 0.09 or x < -0.09:
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

    demo_task_manager = FollowFaceNode()

    demo_task_manager.get_logger().info("DemoTaskManager has started.")
    # demo_task_manager.run()

    rclpy.spin(demo_task_manager)

    demo_task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
