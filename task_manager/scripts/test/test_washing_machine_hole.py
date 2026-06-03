#!/usr/bin/env python3

"""
Test for the washing-machine hole detection via moondream point.

Continuously calls VisionTasks.get_washing_machine_hole_point() and publishes
the resulting 3D point as a Marker in CAMERA_FRAME so it can be visualized in
RViz. The trash_detection_node also publishes a debug image on
/vision/moondream_point_3d_debug overlaying the pixel used for deprojection.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from frida_constants.vision_constants import CAMERA_FRAME
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.task import Task

MARKER_TOPIC = "/vision/test/washing_machine_hole_marker"
CAMERA_FLIP = False  # set True if the camera is mounted upside down
LOOP_RATE_S = 0.5  # how often to re-query (moondream is not fast)


class TestWashingMachineHole(Node):
    def __init__(self):
        super().__init__("test_washing_machine_hole")
        self.vision_manager = VisionTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.vision_manager.camera_upside_down(CAMERA_FLIP)

        Logger.info(
            self,
            f"Point the camera at the washing machine. flip={CAMERA_FLIP}. "
            f"Marker -> '{MARKER_TOPIC}'. "
            f"Debug image -> '/vision/moondream_point_3d_debug'. Ctrl+C to stop.",
        )
        self.run()

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                point = self.vision_manager.get_washing_machine_hole_point()
                if point is None:
                    Logger.warn(self, "No washing machine hole found")
                    self._sleep(LOOP_RATE_S)
                    continue

                Logger.success(
                    self,
                    f"Hole 3D @ {CAMERA_FRAME}: "
                    f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})",
                )
                self._publish_marker(point)
                self._sleep(LOOP_RATE_S)

        except KeyboardInterrupt:
            pass
        finally:
            self.vision_manager.camera_upside_down(False)
            Logger.info(self, "washing machine hole test stopped")

    def _publish_marker(self, point_stamped):
        marker = Marker()
        marker.header.frame_id = CAMERA_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "washing_machine_hole_test"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_stamped.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _sleep(self, seconds: float):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TestWashingMachineHole()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
