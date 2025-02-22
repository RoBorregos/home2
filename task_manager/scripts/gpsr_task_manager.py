#!/usr/bin/env python3

"""
Task Manager to handle Q&A
"""

import rclpy
from config.hri.debug import config as test_hri_config
from frida_interfaces.msg import CommandList
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks


class GPSRTaskManager(Node):
    def __init__(self):
        """Initialize the node"""
        super().__init__("demo_task_manager")
        self.hri = HRITasks(self, config=test_hri_config)
        self.command_queue = CommandList()
        self.command_queue.commands = []
        self._sub = self.create_subscription(
            CommandList, "/task_manager/commands", self.commands_callback, 10
        )
        self.first_run = True
        self.create_timer(0.1, self.run)

    def commands_callback(self, commands_input: CommandList) -> None:
        """Receive processed commands from the interpreter and call executions from the queue"""

        if commands_input:
            self.get_logger().info("Received commands")
        else:
            self.get_logger().info("No commands received")
            return

        for command in commands_input.commands:
            self.get_logger().info(
                f"Added command to queue: {command.action} -> {command.complement} : {command.characteristic}"
            )
            self.command_queue.commands.append(command)

    def run(self) -> None:
        if self.first_run:
            self.hri.say("Hi, I am Frida, your personal assistant. How can I help you today?")
            self.first_run = False
        elif len(self.command_queue.commands) > 0:
            command = self.command_queue.commands[0]
            self.hri.execute_command(command.action, command.complement, command.characteristic)
            self.command_queue.commands.pop(0)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GPSRTaskManager())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
