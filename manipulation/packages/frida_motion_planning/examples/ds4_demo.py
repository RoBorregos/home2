#!/usr/bin/env python3

from sensor_msgs.msg import Joy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.action import ActionClient
from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import Speak
from geometry_msgs.msg import TwistStamped
from xarm_msgs.srv import SetDigitalIO
import rclpy

from frida_constants.hri_constants import (
    SPEAK_SERVICE,
)

from frida_motion_planning.utils.service_utils import move_joint_positions

POS0 = {
    "joints": {
        "joint1": -90.0,
        "joint2": -45.0,
        "joint3": -90.0,
        "joint4": -170.0,
        "joint5": -45.0,
        "joint6": -55.0,
    },
    "degrees": True,
}

POS1 = {
    "joints": {
        "joint1": 30.0,
        "joint2": -10.0,
        "joint3": -40.0,
        "joint4": -170.0,
        "joint5": 45.0,
        "joint6": -55.0,
    },
    "degrees": True,
}


class ServoDS4(Node):
    def __init__(self):
        super().__init__("servo_ds4")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(Joy, "/joy", self.joy_callback, 1)
        self.get_logger().info("Servo DS4 node starting...")
        # Create a client for the service
        self.gripper_client = self.create_client(
            SetDigitalIO, "/xarm/set_tgpio_digital", callback_group=self.callback_group
        )
        self._move_joints_action_client = ActionClient(
            self,
            MoveJoints,
            "/manipulation/move_joints_action_server",
            callback_group=self.callback_group,
        )
        self.speak_service = self.create_client(Speak, SPEAK_SERVICE)
        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/servo_server/delta_twist_cmds",
            QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
        )
        self.get_logger().info("Servo DS4 node started")
        self.busy_planner = False
        self.busy_gripper = False
        self.speak_busy = False
        print("Initialized")

    def joy_callback(self, msg):
        if msg.buttons[0]:
            print("Setting gripper state open")
            self.set_gripper_state("open")
        elif msg.buttons[2]:
            print("Setting gripper state close")
            self.set_gripper_state("close")
        elif msg.buttons[1]:
            print("Setting joint state 0")
            self.send_goal(joint_positions=POS0)
        elif msg.buttons[3]:
            print("Setting joint state 1")
            self.send_goal(joint_positions=POS1)
        elif msg.buttons[4]:
            print("Saying Congratulations")
            self.speak("Congratulations")

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

    # let the server pick the default values
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
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.busy_planner = False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = ServoDS4()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
