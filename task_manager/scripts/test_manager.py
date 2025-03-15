#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy

# from config.hri.debug import config as test_hri_config
from rclpy.node import Node

# from subtask_managers.hri_tasks import HRITasks
from subtask_managers.manipulation_tasks import ManipulationTasks


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = {}
        # self.subtask_manager["hri"] = HRITasks(self, config=test_hri_config)

        self.subtask_manager["manipulation"] = ManipulationTasks(self, task="DEMO", mock_data=False)

        # wait for a bit
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        """testing vision tasks"""

        # self.subtask_manager["hri"].say(
        #     "Hi, my name is frida. What is your favorite drink?", wait=True
        # )
        # self.get_logger().info("Hearing from the user...")

        # # This line does run
        # user_request = self.subtask_manager["hri"].hear()

        # self.get_logger().info(f"Heard: {user_request}")

        # drink = self.subtask_manager["hri"].extract_data("Drink", user_request)

        # self.get_logger().info(f"Extracted data: {drink}")

        # commands = self.subtask_manager["hri"].command_interpreter(user_request)

        # self.get_logger().info(f"Interpreted commands: {commands}")

        # command_strs = [
        #     f"I will do action:{command.action}, ({command.complement}), ({command.characteristic})"
        #     for command in commands
        # ]
        # command_str = " and ".join(command_strs)

        # fixed_text = self.subtask_manager["hri"].refactor_text(command_str)
        # self.subtask_manager["hri"].say(fixed_text)

        # self.subtask_manager["hri"].say("I'm frida, Can you tell me where to go?")
        # location_hint = self.subtask_manager["hri"].hear()

        # joint_positions = self.subtask_manager["manipulation"].get_joint_positions()
        # joint_positions["joint1"] = joint_positions["joint1"] - 180
        # print(joint_positions)
        # new_joint_positions = [-55.0, -3.0, -52.0, 0.0, 53.0, -55.0]
        # res = self.subtask_manager["manipulation"].move_joint_positions(
        #     joint_positions=joint_positions, velocity=0.5, degrees=True
        # )
        # print("Move joint positions result: ", res)
        # joint_positions = self.subtask_manager["manipulation"].get_joint_positions(degrees=True)
        # print(joint_positions)

        ####### EXAMPLE: Move to named position then move only the first joint #######
        # Move to a named position
        joint_positions = self.subtask_manager["manipulation"].get_joint_positions()
        print(joint_positions)
        res = self.subtask_manager["manipulation"].move_joint_positions(
            named_position="front_stare", velocity=0.5, degrees=True
        )
        print("Move to named position result: ", res)
        joint_positions = self.subtask_manager["manipulation"].get_joint_positions(degrees=True)
        joint_positions["joint1"] = joint_positions["joint1"] - 45
        res = self.subtask_manager["manipulation"].move_joint_positions(
            joint_positions=joint_positions, velocity=0.5, degrees=True
        )
        print("Move joint positions result: ", res)


def main(args=None):
    rclpy.init(args=args)
    node = TestTaskManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
