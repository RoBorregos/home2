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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from task_manager.utils.logger import Logger
from task_manager.utils.subtask_manager import SubtaskManager, Task

# Position to receive/pick up the diploma (front stare)
POS_RECEIVE = {
    "joint1": -90.0,
    "joint2": -45.0,
    "joint3": -90.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 45.0,
}

# Position to deliver the diploma (rotated on joint1 to face the person)
POS_DELIVER = {
    "joint1": 0.0,
    "joint2": -45.0,
    "joint3": -90.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 45.0,
}


class DemoBecas(Node):
    def __init__(self):
        super().__init__("demo_becas")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.subtask_manager = SubtaskManager(self, task=Task.DEMO)

        self.create_subscription(
            Joy, "/joy", self.joy_callback, 1, callback_group=self.callback_group
        )

        self.busy = False

        Logger.info(self, "Demo Becas node starting...")
        Logger.info(self, "Demo Becas ready! Use PlayStation controller buttons.")
        Logger.info(self, "  Square:   Open gripper")
        Logger.info(self, "  Circle:   Close gripper")
        Logger.info(self, "  X:        Move to RECEIVE position")
        Logger.info(self, "  Triangle: Move to DELIVER position")
        Logger.info(self, "  L1:       Say Congratulations")

    def joy_callback(self, msg):
        if self.busy:
            return

        if msg.buttons[0]:  # Square
            self.busy = True
            Logger.info(self, "Opening gripper")
            self.subtask_manager.manipulation.open_gripper()
            self.busy = False
        elif msg.buttons[2]:  # Circle
            self.busy = True
            Logger.info(self, "Closing gripper")
            self.subtask_manager.manipulation.close_gripper()
            self.busy = False
        elif msg.buttons[1]:  # X
            self.busy = True
            Logger.info(self, "Moving to RECEIVE position")
            self.subtask_manager.manipulation.move_joint_positions(
                joint_positions=POS_RECEIVE, velocity=0.3, degrees=True
            )
            self.busy = False
        elif msg.buttons[3]:  # Triangle
            self.busy = True
            Logger.info(self, "Moving to DELIVER position")
            self.subtask_manager.manipulation.move_joint_positions(
                joint_positions=POS_DELIVER, velocity=0.3, degrees=True
            )
            self.busy = False
        elif msg.buttons[4]:  # L1
            self.busy = True
            Logger.info(self, "Saying Congratulations")
            self.subtask_manager.hri.say("Congratulations! You did an amazing job!")
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = DemoBecas()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
