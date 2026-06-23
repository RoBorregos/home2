#!/usr/bin/env python3
"""
Node exposing a service to filter YOLO detections by label_text.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    OBJECT_POINTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
    CAMERA_TOPIC,
    TRASHCAN_SERVICE,
    MOONDREAM_POINT_3D_TOPIC,
    MOONDREAM_POINT_3D_DEBUG_TOPIC,
    MOONDREAM_BBOX_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import (
    ObjectPoints,
    TrashcanDetection,
    MoondreamPoint3D,
    MoondreamObjectBBox,
)
from vision_general.utils.calculations import get_depth, deproject_pixel_to_point
from vision_general.utils.ros_utils import wait_for_future

# Minimum number of valid depth pixels in the ROI to trust the 3D centroid.
MIN_VALID_DEPTH_PIXELS = 30

TRASH_DEBUG_IMAGE_TOPIC = "/vision/trash_detection_debug"


class TrashDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("trash_detection_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.image = None
        self.imageInfo = None
        self.depth_image = None
        self.trash_debug_image = None

        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.create_service(
            TrashcanDetection,
            TRASHCAN_SERVICE,
            self.get_trashcan,
            callback_group=self.callback_group,
        )

        self.trashcan_pub = self.create_publisher(
            ObjectDetectionArray, DETECTIONS_TOPIC, 10
        )

        self.debug_image_pub = self.create_publisher(Image, TRASH_DEBUG_IMAGE_TOPIC, 10)

        self.moondream_point_3d_debug_pub = self.create_publisher(
            Image, MOONDREAM_POINT_3D_DEBUG_TOPIC, 10
        )

        self.moondream_point_client = self.create_client(
            ObjectPoints, OBJECT_POINTS_TOPIC
        )
        self.moondream_bbox_client = self.create_client(
            MoondreamObjectBBox, MOONDREAM_BBOX_TOPIC
        )

        self.create_service(
            MoondreamPoint3D,
            MOONDREAM_POINT_3D_TOPIC,
            self.get_moondream_point_3d,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Trash service ready")

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.trash_debug_image is not None:
                self.debug_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(self.trash_debug_image, "bgr8")
                )
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def image_info_callback(self, data):
        self.imageInfo = data

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def get_trashcan(self, req, res):
        if self.imageInfo is None:
            self.get_logger().warn("Cannot detect trashcan without camera info")
            return []

        if self.depth_image is None:
            self.get_logger().warn("Cannot detect trashcan without depth image")
            return

        trashcan_pts = self.get_moondream_points("black trashcan")

        if not trashcan_pts:
            self.get_logger().warn("No trashcan detected")
            return

        trashcans = ObjectDetectionArray()
        debug_image = None
        for pt in trashcan_pts:
            self.get_logger().info("Detected trashcan")
            trashcan = ObjectDetection()
            ptStamped, point2d = self._deproject_normalized(pt[0], pt[1])

            trashcan.label_text = "trashcan"
            trashcan.point3d = ptStamped
            trashcans.detections.append(trashcan)

            if self.image is not None and debug_image is None:
                debug_image = self.image.copy()
                u, v = point2d
                cv2.circle(debug_image, (u, v), 12, (255, 0, 0), -1)
                cv2.circle(debug_image, (u, v), 20, (255, 255, 255), 2)
                cv2.putText(
                    debug_image,
                    "trashcan",
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2,
                )

        self.trashcan_pub.publish(trashcans)
        res.detection_array = trashcans
        res.success = True

        if debug_image is not None:
            self.trash_debug_image = debug_image
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

        return res

    def build_point_stamped(self, xyz):
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = CAMERA_FRAME
        point_stamped.point.x = float(xyz[2])
        point_stamped.point.y = float(-xyz[0])
        point_stamped.point.z = float(-xyz[1])
        return point_stamped

    def _deproject_normalized(self, nx, ny):
        point2d = (
            min(max(int(nx * self.imageInfo.width), 0), self.imageInfo.width - 1),
            min(max(int(ny * self.imageInfo.height), 0), self.imageInfo.height - 1),
        )
        depth = get_depth(self.depth_image, point2d)
        point3d = deproject_pixel_to_point(self.imageInfo, point2d, depth)
        return self.build_point_stamped(point3d), point2d

    def _deproject_normalized_optical(self, nx, ny):
        """Deproject one normalized pixel and return a PointStamped in the
        raw optical-frame convention (x=right, y=down, z=forward).
        """
        # ZED publishes camera_info at the RGB resolution but the depth image
        # can be downsampled; clamp to whichever is smaller.
        depth_h, depth_w = self.depth_image.shape[:2]
        max_w = min(self.imageInfo.width, depth_w) - 1
        max_h = min(self.imageInfo.height, depth_h) - 1
        point2d = (
            min(max(int(nx * self.imageInfo.width), 0), max_w),
            min(max(int(ny * self.imageInfo.height), 0), max_h),
        )
        # calculations.get_depth swaps axes internally — pass (row, col).
        depth = get_depth(self.depth_image, (point2d[1], point2d[0]))
        point3d = deproject_pixel_to_point(self.imageInfo, point2d, depth)
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = CAMERA_FRAME
        ps.point.x = float(point3d[0])
        ps.point.y = float(point3d[1])
        ps.point.z = float(point3d[2])
        return ps, point2d

    def get_moondream_point_3d(self, req, res):
        """Ask moondream for a bbox, deproject every valid depth pixel inside
        it, and return the mean of that pointcloud in `CAMERA_FRAME`.
        Returns `success=False` (no silent fallback) and publishes a red
        debug frame when the bbox is missing or the ROI has too few depth
        samples.
        """
        res.success = False
        if self.imageInfo is None or self.depth_image is None:
            self.get_logger().warn("Centroid request before camera_info / depth ready")
            return res

        bbox = self._call_moondream_bbox(req.subject)
        if bbox is None:
            self._publish_failure_debug(req.subject, "NO DETECTION")
            return res

        depth_h, depth_w = self.depth_image.shape[:2]
        w = min(self.imageInfo.width, depth_w)
        h = min(self.imageInfo.height, depth_h)
        xmin = int(max(0, min(w - 1, bbox["xmin"] * w)))
        ymin = int(max(0, min(h - 1, bbox["ymin"] * h)))
        xmax = int(max(xmin + 1, min(w, bbox["xmax"] * w)))
        ymax = int(max(ymin + 1, min(h, bbox["ymax"] * h)))

        roi = self.depth_image[ymin:ymax, xmin:xmax]
        valid = (roi > 0) & np.isfinite(roi)
        n_valid = int(np.count_nonzero(valid))
        if n_valid < MIN_VALID_DEPTH_PIXELS:
            self.get_logger().warn(
                f"Only {n_valid} valid depth pixels in ROI "
                f"({xmin},{ymin})->({xmax},{ymax}); need {MIN_VALID_DEPTH_PIXELS}."
            )
            self._publish_failure_debug(
                req.subject,
                f"INSUFFICIENT DEPTH (n={n_valid})",
                bbox_px=(xmin, ymin, xmax, ymax),
            )
            return res

        v_local, u_local = np.where(valid)
        z = roi[v_local, u_local].astype(np.float64)
        u_global = u_local.astype(np.float64) + xmin
        v_global = v_local.astype(np.float64) + ymin
        fx, fy = self.imageInfo.k[0], self.imageInfo.k[4]
        cx_i, cy_i = self.imageInfo.k[2], self.imageInfo.k[5]
        X = (u_global - cx_i) * z / fx
        Y = (v_global - cy_i) * z / fy
        Xc, Yc, Zc = float(np.mean(X)), float(np.mean(Y)), float(np.mean(z))

        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = CAMERA_FRAME
        ps.point.x = Xc
        ps.point.y = Yc
        ps.point.z = Zc
        res.point = ps
        res.success = True

        u_centroid = int(round(Xc * fx / Zc + cx_i)) if Zc > 0 else (xmin + xmax) // 2
        v_centroid = int(round(Yc * fy / Zc + cy_i)) if Zc > 0 else (ymin + ymax) // 2
        self.get_logger().info(
            f"Moondream 3D centroid for '{req.subject}': "
            f"({Xc:.3f}, {Yc:.3f}, {Zc:.3f}) from {n_valid} px in ROI "
            f"({xmin},{ymin})->({xmax},{ymax})"
        )
        self.publish_moondream_point_3d_debug(
            req.subject, (u_centroid, v_centroid), ps, (xmin, ymin, xmax, ymax), n_valid
        )
        return res

    def _call_moondream_bbox(self, subject):
        """Return the bbox dict from `/vision/moondream_bbox`, or None."""
        if not self.moondream_bbox_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("moondream_bbox service not available")
            return None
        req = MoondreamObjectBBox.Request()
        req.subject = subject
        future = self.moondream_bbox_client.call_async(req)
        future = wait_for_future(future, 15)
        if future is False or not future.done():
            self.get_logger().error("moondream_bbox call timed out")
            return None
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(f"moondream_bbox: no bbox for '{subject}'")
            return None
        return {
            "xmin": float(result.xmin),
            "ymin": float(result.ymin),
            "xmax": float(result.xmax),
            "ymax": float(result.ymax),
        }

    def _publish_failure_debug(self, subject, message, bbox_px=None):
        if self.image is None:
            return
        debug = self.image.copy()
        if bbox_px is not None:
            x0, y0, x1, y1 = bbox_px
            cv2.rectangle(debug, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.putText(
            debug,
            f"{message}: {subject[:50]}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )
        try:
            self.moondream_point_3d_debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, "bgr8")
            )
        except Exception as e:
            self.get_logger().error(f"Debug publish error: {e}")

    def publish_moondream_point_3d_debug(
        self, subject, center_px, point_stamped, bbox_px=None, n_valid=None
    ):
        if self.image is None:
            return

        debug = self.image.copy()
        if bbox_px is not None:
            x0, y0, x1, y1 = bbox_px
            cv2.rectangle(debug, (x0, y0), (x1, y1), (0, 255, 0), 2)
        u, v = center_px
        cv2.drawMarker(debug, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(debug, (u, v), 14, (0, 255, 0), 2)
        label_parts = [
            f"{subject[:40]}",
            f"xyz=({point_stamped.point.x:.2f},{point_stamped.point.y:.2f},"
            f"{point_stamped.point.z:.2f})",
        ]
        if n_valid is not None:
            label_parts.append(f"N={n_valid}")
        cv2.putText(
            debug,
            "  ".join(label_parts),
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
        )

        try:
            self.moondream_point_3d_debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, "bgr8")
            )
        except Exception as e:
            self.get_logger().error(f"Debug image publish error: {e}")

    def get_moondream_points(self, subject) -> list[tuple[float, float]]:
        """Get object points from the MoonDream service."""
        req = ObjectPoints.Request()
        req.subject = subject

        future = self.moondream_point_client.call_async(req)
        future = wait_for_future(future, 15)

        if future is False or not future.done():
            self.get_logger().error("MoonDream service call timed out or failed")
            return []

        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("MoonDream table point detection failed")
            return []

        return [(p.x, p.y) for p in result.points]


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
