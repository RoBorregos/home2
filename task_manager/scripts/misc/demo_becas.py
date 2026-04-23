#!/usr/bin/env python3

"""
Demo Becas - PlayStation controller operated diploma delivery demo.

Button mapping (DualShock/DualSense):
  Square  (0): Open gripper
  Circle  (2): Close gripper
  X       (1): Move arm to RECEIVE position (front stare)
  Triangle(3): Move arm to DELIVER position (rotated to hand diploma)
  L1      (4): Say "Congratulations"
"""

from sensor_msgs.msg import Joy

from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import Speak
from xarm_msgs.srv import SetDigitalIO
import rclpy

from frida_constants.hri_constants import SPEAK_SERVICE
from frida_motion_planning.utils.service_utils import move_joint_positions

# Position to receive/pick up the diploma (front stare)
POS_RECEIVE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -45.0,
        "joint3": -90.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6": 45.0,
    },
    "degrees": True,
}

# Position to deliver the diploma (rotated on joint1 to face the person)
POS_DELIVER = {
    "joints": {
        "joint1": 0.0,
        "joint2": -45.0,
        "joint3": -90.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6": 45.0,
    },
    "degrees": True,
}


class DemoBecas(Node):
    def __init__(self):
        super().__init__("demo_becas")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(
            Joy, "/joy", self.joy_callback, 1, callback_group=self.callback_group
        )
        self.get_logger().info("Demo Becas node starting...")

        self.gripper_client = self.create_client(
            SetDigitalIO,
            "/xarm/set_tgpio_digital",
            callback_group=self.callback_group,
        )
        self._move_joints_action_client = ActionClient(
            self,
            MoveJoints,
            "/manipulation/move_joints_action_server",
            callback_group=self.callback_group,
        )
        self.speak_service = self.create_client(
            Speak, SPEAK_SERVICE, callback_group=self.callback_group
        )

        self.busy_planner = False
        self.busy_gripper = False
        self.speak_busy = False

        self.get_logger().info("Demo Becas ready! Use PlayStation controller buttons.")
        self.get_logger().info("  Square:   Open gripper")
        self.get_logger().info("  Circle:   Close gripper")
        self.get_logger().info("  X:        Move to RECEIVE position")
        self.get_logger().info("  Triangle: Move to DELIVER position")
        self.get_logger().info("  L1:       Say Congratulations")

    def joy_callback(self, msg):
        if msg.buttons[0]:  # Square
            self.get_logger().info("Opening gripper")
            self.set_gripper_state("open")
        elif msg.buttons[2]:  # Circle
            self.get_logger().info("Closing gripper")
            self.set_gripper_state("close")
        elif msg.buttons[1]:  # X
            self.get_logger().info("Moving to RECEIVE position")
            self.send_goal(joint_positions=POS_RECEIVE)
        elif msg.buttons[3]:  # Triangle
            self.get_logger().info("Moving to DELIVER position")
            self.send_goal(joint_positions=POS_DELIVER)
        elif msg.buttons[4]:  # L1
            self.get_logger().info("Saying Congratulations")
            self.speak("Congratulations! You did an amazing job!")

    def speak(self, text):
        if self.speak_busy:
            return
        request = Speak.Request(text=text)
        future = self.speak_service.call_async(request)
        self.speak_busy = True
        future.add_done_callback(self.speak_callback)

    def speak_callback(self, future):
        self.speak_busy = False

    def set_gripper_state(self, state):
        if self.busy_gripper:
            return
        request = SetDigitalIO.Request()
        request.ionum = int(0)
        request.value = int(0) if state == "open" else int(1)
        future = self.gripper_client.call_async(request)
        self.busy_gripper = True
        future.add_done_callback(self.gripper_callback)

    def send_goal(self, joint_positions):
        if self.busy_planner:
            return
        future = move_joint_positions(
            move_joints_action_client=self._move_joints_action_client,
            joint_positions=joint_positions,
            velocity=0.3,
            wait=False,
        )
        future.add_done_callback(self.goal_response_callback)
        self.busy_planner = True

    def gripper_callback(self, future):
        self.busy_gripper = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.busy_planner = False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = DemoBecas()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
