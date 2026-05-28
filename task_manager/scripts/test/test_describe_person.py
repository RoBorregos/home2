#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import Task
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.subtask_managers.hri_tasks import HRITasks


class TestPersonDescription(Node):
    def __init__(self):
        super().__init__("describe_person_test")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.hri = HRITasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.result = {"status": None, "description": ""}

    def on_description(self, status, description):
        self.result["status"] = status

        if status == Status.EXECUTION_SUCCESS and description:
            future = self.hri.answer_with_context(
                question=(
                    "Convert these physical attributes into a single fluent English sentence "
                    "suitable for spoken speech. Start with 'They are'. "
                    "Do not use semicolons or list formatting."
                ),
                context=description,
                is_async=True,
            )

            def callback(f):
                _, natural = f.result()
                natural_answer = natural if natural else description
                self.result["description"] = natural_answer
                print(f"Final natural description: {natural_answer}")
                self.hri.say(natural_answer, wait=False)

            future.add_done_callback(callback)
        else:
            self.result["description"] = description
            print(f"Raw description: {description}")

    def run_test(self):
        print("Starting person description test...")
        self.vision.describe_person(callback=self.on_description)

        start_time = time.time()
        # Spin for up to 30 seconds to allow async operations to complete
        while time.time() - start_time < 30:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.result["description"]:
                break

        print(f"Test finished. Result description: {self.result['description']}")


def main(args=None):
    rclpy.init(args=args)
    node = TestPersonDescription()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
