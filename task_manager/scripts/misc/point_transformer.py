#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node
import rclpy.qos
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

# from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.srv import (
    PointTransformation,
    ReturnLocation,
    LaserGet,
    GetAreaForPoint,
)
import json
import os
import time
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from task_manager.utils.status import Status
from math import sqrt

from frida_constants.integration_constants import (
    POINT_TRANSFORMER_TOPIC,
    RETURN_LOCATION,
    RETURN_LASER_DATA,
)
from frida_constants.navigation_constants import GET_AREA_FOR_POINT_SERVICE
from geometry_msgs.msg import PointStamped


class PointTransformer(Node):
    def __init__(self):
        super().__init__("point_transformer")
        # Create a callback group for concurrent callbacks
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        # TF2 setup
        self.laser_sub = None
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos)

        self.set_target_service = self.create_service(
            PointTransformation, POINT_TRANSFORMER_TOPIC, self.set_target_callback
        )

        self.return_areas = self.create_service(
            ReturnLocation, RETURN_LOCATION, self.get_current_location
        )

        self.return_laser = self.create_service(LaserGet, RETURN_LASER_DATA, self.send_laser_data)

        # nav_central owns the area polygons of the ACTIVE map; this node used to
        # carry its own point-in-polygon over the static frida_constants copy,
        # which belongs to a different arena. Ask nav, fall back to the local copy
        # only when navigation is not running.
        self.area_client = self.create_client(GetAreaForPoint, GET_AREA_FOR_POINT_SERVICE)

        self.scan_topic = self.create_subscription(
            LaserScan, "/scan", self.update_laser, qos_profile=qos
        )
        self.get_logger().info("PointTransformer node has been started.")

        # self.get_actual_pose()

    def update_laser(self, msg: LaserScan):
        self.laser_sub = msg

    def set_target_callback(self, request, response):
        """Convert the object to height"""
        try:
            # First check if transform is available with timeout
            if not self.tf_buffer.can_transform(
                request.target_frame,
                request.point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0),
            ):
                self.get_logger().error(
                    self, "Transform from camera to base_link not available yet"
                )
                response.success = False
                response.message = "Transform from camera to base_link not available yet"
                return response

            transform_frame = self.tf_buffer.lookup_transform(
                request.target_frame, request.point.header.frame_id, rclpy.time.Time()
            )

            transformed_point = do_transform_point(request.point, transform_frame)
            response.success = True
            response.transformed_point = transformed_point
            return response
        except Exception as e:
            self.get_logger().error(self, f"Error converting to height: {e}")
            response.success = False
            response.message = "Error converting to height: {e}"
            return response

    def get_actual_pose(self):
        try:
            # Wait for the transform to become available (with a timeout)
            self.tf_buffer.can_transform(
                "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )
            rclpy.spin_once(self, timeout_sec=0.2)
            # Get the transform from base_link to map
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Extract the pose information
            posex = transform.transform.translation.x
            posey = transform.transform.translation.y

            self.get_logger().info(f"Robot's position in the map frame: x={posex}, y={posey}")
            self.get_logger().info(f"Robot's transform: {transform.transform}")
            response = ReturnLocation.Response()
            response.posex = posex
            response.posey = posey
            response.status = Status.EXECUTION_SUCCESS
        except Exception as e:
            self.get_logger().info(f"Error getting position: {e}")
            response.status = Status.EXECUTION_ERROR
            return

    def send_laser_data(self, request, response):
        try:
            if self.laser_sub is not None:
                response.data = self.laser_sub
                response.status = True
            else:
                response.status = False
            return response
        except Exception as e:
            self.get_logger().debug(e)
            response.status = False
            return response

    def get_location_from_pose(self, posex, posey):
        """Area and nearest sublocations for a map-frame point.

        Prefers nav_central's `GetAreaForPoint`, which uses the polygons of the
        map that is actually loaded. Falls back to the bundled areas.json when
        navigation is down, so a standalone task_manager still answers.
        """
        if self.area_client.wait_for_service(timeout_sec=1.0):
            request = GetAreaForPoint.Request()
            request.point = PointStamped()
            request.point.header.frame_id = "map"
            request.point.point.x = float(posex)
            request.point.point.y = float(posey)

            # Never spin from inside a callback: the executor's other threads
            # complete the future (see utils/area_check.py for the crash this
            # caused in vision).
            future = self.area_client.call_async(request)
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            result = future.result() if future.done() else None

            if result is not None and result.success:
                if not result.area:
                    self.get_logger().warning("I don't know where I am")
                    return None, None
                self.get_logger().info(f"I AM IN: {result.area}")
                return result.area, list(result.sublocations)
            self.get_logger().warning("get_area_for_point failed; using the bundled areas.json")
        else:
            self.get_logger().warning(
                f"{GET_AREA_FOR_POINT_SERVICE} unavailable; using the bundled areas.json"
            )
        return self._get_location_from_pose_local(posex, posey)

    def _get_location_from_pose_local(self, posex, posey):
        """Fallback: the static frida_constants copy of areas.json.

        Kept only for running without navigation — it describes a different map
        than the one nav_central serves, so the answer can disagree.
        """

        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")
        mylocation = ""

        # Load areas from the JSON file
        with open(file_path, "r") as file:
            areas = json.load(file)

        # Check which area the robot is in
        for area in areas:
            polygon = areas.get(area, {}).get("polygon", [])
            # Skip if "polygon" does not exist or is empty
            if not polygon:
                continue

            self.get_logger().info(f"Checking area: {area}")
            if self.is_inside(posex, posey, polygon):
                mylocation = area
                break
        if mylocation == "":
            self.get_logger().warning("I don't know where I am")
            return None, None

        self.get_logger().info(f"I AM IN: {mylocation}")

        places = list(areas[mylocation].keys())
        distances = []
        results = []

        for place in places:
            if len(areas[mylocation][place]) == 0:
                continue
            if place == "polygon":
                continue
            place_x = areas[mylocation][place][0]
            place_y = areas[mylocation][place][1]
            distance = sqrt((pow(place_x - posex, 2)) + (pow(place_y - posey, 2)))
            dist = {"subarea": place, "distance": distance}
            distances.append(dist)
            # Sort the distances in ascending order
        distances.sort(key=lambda d: d["distance"])
        for distance in distances:
            results.append(distance["subarea"])

        return mylocation, results

    def is_inside(self, x, y, polygon):
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if (y1 > y) != (y2 > y):
                xinters = (y - y1) * (x2 - x1) / (y2 - y1 + 1e-10) + x1  # Avoid zero division
                if x < xinters:
                    inside = not inside
        return inside

    def get_current_location(self, request, response):
        """
        Original method to handle the request/response for determining the robot's location.
        """
        try:
            # Wait for the transform to become available (with a timeout)
            self.tf_buffer.can_transform(
                "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )

            # Get the transform from base_link to map
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Extract the pose information
            posex = transform.transform.translation.x
            posey = transform.transform.translation.y

            self.get_logger().info(f"Robot's position in the map frame: x={posex}, y={posey}")

            # Use the callback to get the location
            mylocation, nearest_locations = self.get_location_from_pose(posex, posey)
            if mylocation is None:
                response.location = ""
                response.nearest_locations = []
                response.success = False
                return response

            response.nearest_locations = nearest_locations
            response.location = mylocation
            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"Could not get transform: {str(e)}")
            response.location = ""
            response.success = False
            response.nearest_locations = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()

    # MultiThreadedExecutor is required, not cosmetic: get_current_location is a
    # service callback that itself calls GetAreaForPoint, and a single-threaded
    # executor can never deliver that response while it is blocked in the outer
    # callback.
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
