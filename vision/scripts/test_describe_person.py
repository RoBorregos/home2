#!/usr/bin/env python3

import argparse
import threading
import time

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task


def main() -> int:
    parser = argparse.ArgumentParser(description="Test VisionTasks.describe_person")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("describe_person_test")
    vision = VisionTasks(node, task=Task.HRIC)

    done = threading.Event()
    result = {"status": None, "description": ""}

    def on_description(status, description):
        result["status"] = status
        result["description"] = description
        done.set()

    vision.describe_person(callback=on_description)

    start = time.time()
    try:
        while rclpy.ok() and not done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > args.timeout:
                node.get_logger().error("Timed out waiting for description")
                break
    finally:
        if result["status"] == Status.EXECUTION_SUCCESS:
            print(result["description"])
        elif done.is_set():
            print(f"Status: {result['status']}, description: {result['description']}")
        else:
            print("No result")

        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
