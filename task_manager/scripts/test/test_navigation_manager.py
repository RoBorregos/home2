#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.subtask_managers.nav_tasks import NavigationTasks


class TestNavigationManager(Node):
    def __init__(self):
        super().__init__("NavigationTaskManager")

        self.navigation_manager = NavigationTasks(self, task=Task.DEBUG, mock_data=False)
        
        self.tests_funcs = {
                "check_door": "",
                "retrieve_areas": "" 
                }

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Test Navigation Manager started.")
        self.run()
    
    def check_functions(self, command):
        Logger.info(self,f"Testing '{command}' command")
        if hasattr(self.navigation_manager, command):
            result = getattr(self.navigation_manager, command)()
            Logger.info(self,f"Result = {result}")
        else:
            Logger.warn(self,f"Test '{command}' not found")
             

    def run(self): 
        for command in self.tests_funcs:
            self.check_functions(command)

def main(args=None):
    rclpy.init(args=args)
    node = TestNavigationManager()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
