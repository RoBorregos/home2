#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import os
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import HeatmapPlace
from scipy.signal import convolve2d
from frida_constants.manipulation_constants import (
    HEATMAP_PLACE_SERVICE,
    PLACE_MAX_DISTANCE,
    PLACE_POINT_DEBUG_TOPIC,
)
import json
from place.heatmap_generators.close_by_generators import (
    generate_close_to_heatmap,
    generate_directional_heatmap,
)

# change matplotlib backend to avoid X server dependency
import matplotlib

matplotlib.use("Agg")  # Use a non-interactive backend

CLOSE_BY_MAX_DISTANCE = 0.3


class HeatmapServer(Node):
    def __init__(self):
        super().__init__("heatmap_service")
        self.save_image = True
        self.srv = self.create_service(
            HeatmapPlace, HEATMAP_PLACE_SERVICE, self.handle_heatmap
        )
        self.place_point_debug_point_publisher = self.create_publisher(
            PointStamped, PLACE_POINT_DEBUG_TOPIC, 10
        )
        self.get_logger().info("Heatmap Place Service ready")

    def handle_heatmap(self, request, response):
        # Convert PointCloud2 to numpy array
        point_cloud = request.pointcloud
        prefer_closest = request.prefer_closest

        request_dict = None
        if request.special_request != "":
            try:
                # Attempt to parse the special request as JSON
                request_dict = json.loads(request.special_request)
            except json.JSONDecodeError:
                self.get_logger().error(
                    f"Invalid JSON in special request: {request.special_request}"
                )
                response.place_point = PointStamped()

        if point_cloud.header.frame_id != "base_link":
            self.get_logger().warn(
                f"PointCloud2 frame_id is {point_cloud.header.frame_id}, expected base_link"
            )
            return response
        cloud_gen = point_cloud2.read_points(
            point_cloud, field_names=("x", "y", "z"), skip_nans=True
        )

        max_distance = PLACE_MAX_DISTANCE
        cloud_points = []
        filtered_cloud_points = []
        self.get_logger().info("Filtering point cloud")
        for p in cloud_gen:
            if np.sqrt(p[0] ** 2 + p[1] ** 2) > max_distance:
                filtered_cloud_points.append([p[0], p[1], p[2]])
                continue
            cloud_points.append([p[0], p[1], p[2]])
        self.get_logger().info(
            f"Filtered {len(filtered_cloud_points)} out of {len(cloud_points)} points"
        )
        cloud_points = np.array(cloud_points)
        filtered_cloud_points = np.array(filtered_cloud_points)

        # If no points found, return zero point
        if len(cloud_points) == 0:
            self.get_logger().warn("Empty pointcloud received")
            response.place_point = PointStamped()
            return response

        self.grid_size = 0.015  # meters
        self.grid_size_mm = int(self.grid_size * 1000)

        # Convert points to mm coordinates
        self.x_mm = (cloud_points[:, 0] * 1000).astype(int)
        self.y_mm = (cloud_points[:, 1] * 1000).astype(int)

        # Create histogram bins
        x_range = np.arange(
            min(self.x_mm), max(self.x_mm) + self.grid_size_mm, self.grid_size_mm
        )
        y_range = np.arange(
            min(self.y_mm), max(self.y_mm) + self.grid_size_mm, self.grid_size_mm
        )
        hist, xedges, yedges = np.histogram2d(
            self.x_mm, self.y_mm, bins=[x_range, y_range]
        )
        self.min_x_mm = xedges[0]
        self.min_y_mm = yedges[0]

        # Generate binary map
        binary_map = hist > 0

        # Create heat/cool kernels
        heat_kernel_length = 0.3  # meters
        cool_kernel_length = 0.15  # meters
        heat_multiplier = 1.0
        cool_multiplier = 10.0

        # Heat kernel
        heat_kernel_size = int(heat_kernel_length / self.grid_size)
        xx, yy = np.mgrid[
            -heat_kernel_size // 2 : heat_kernel_size // 2 + 1,
            -heat_kernel_size // 2 : heat_kernel_size // 2 + 1,
        ]
        heat_kernel = np.exp(-(xx**2 + yy**2) / (2 * (heat_kernel_size / 2) ** 2))
        heat_kernel *= heat_multiplier

        # Cool kernel
        cool_kernel_size = int(cool_kernel_length / self.grid_size)
        xx, yy = np.mgrid[
            -cool_kernel_size // 2 : cool_kernel_size // 2 + 1,
            -cool_kernel_size // 2 : cool_kernel_size // 2 + 1,
        ]
        cool_kernel = np.exp(-(xx**2 + yy**2) / (2 * (cool_kernel_size / 2) ** 2))
        cool_kernel *= cool_multiplier

        heat_map = convolve2d(binary_map.astype(float), heat_kernel, mode="same")
        cool_map = convolve2d((~binary_map).astype(float), cool_kernel, mode="same")

        # Combine maps
        final_map = heat_map - cool_map
        # final_map = final_map * closeness_map
        final_map[~binary_map] = 0
        final_map = np.clip(final_map, 0, None)

        # if prefer closest, multiply everything by how close it is to 0,0
        closeness_map = None
        if prefer_closest:
            self.get_logger().info("Prefer closest point")
            closeness_map = np.zeros_like(binary_map, dtype=float)
            for i in range(closeness_map.shape[0]):
                for j in range(closeness_map.shape[1]):
                    x = (self.min_x_mm + (i + 0.5) * self.grid_size_mm) / 1000.0
                    y = (self.min_y_mm + (j + 0.5) * self.grid_size_mm) / 1000.0
                    closeness_map[i, j] = np.sqrt(x**2 + y**2)
            closeness_map = np.exp(-closeness_map / (self.grid_size_mm * 2))
            # normalize so smallest value is 0 and largest is 1
            closeness_map = (closeness_map - np.min(closeness_map)) / (
                np.max(closeness_map) - np.min(closeness_map)
            )
            closeness_map = (
                closeness_map**3
            )  # make the decay from close to far sharper -> prefer even closer points
            final_map = final_map * closeness_map

            # Normalize final map
            final_map = final_map / np.max(final_map)
            final_map = np.clip(final_map, 0, None)

        special_request_map = self.generate_special_request_map(
            binary_map, request, request_dict
        )

        if special_request_map is not None:
            self.get_logger().info("Applying special request map")
            final_map = final_map * special_request_map

        object_point = request.close_point

        if object_point.header.frame_id != "":
            self.get_logger().info(
                f"Close point, frame_id: {object_point.header.frame_id}"
            )
            self.get_logger().info("Generating close to heatmap")
            close_to_map = self.generate_close_to_heatmap(
                binary_map,
                [object_point.point.x, object_point.point.y],
                min_x_mm,
                min_y_mm,
                grid_size_mm,
            )
            final_map = final_map * close_to_map

        # Find best position
        max_idx = np.unravel_index(np.argmax(final_map), final_map.shape)

        x_center = (self.min_x_mm + (max_idx[0] + 0.5) * self.grid_size_mm) / 1000.0
        y_center = (self.min_y_mm + (max_idx[1] + 0.5) * self.grid_size_mm) / 1000.0

        # get the closest point to this one and extract its z coordinate
        closest_point = None
        min_distance = float("inf")
        self.get_logger().info("Finding closest point")
        for point in cloud_points:
            distance = np.sqrt((point[0] - x_center) ** 2 + (point[1] - y_center) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_point = point
        self.get_logger().info(f"Closest point: {closest_point}")
        z_center = float(closest_point[2])
        # Create response
        response.place_point = PointStamped()
        response.place_point.header = point_cloud.header
        response.place_point.point.x = x_center
        response.place_point.point.y = y_center
        response.place_point.point.z = z_center

        # Publish the point
        self.get_logger().info(f"Publishing point: {response.place_point}")
        self.place_point_debug_point_publisher.publish(response.place_point)

        # Save visualization
        if self.save_image:
            try:
                plt.figure(figsize=(16, 8)) # Parameter changed to 8, prev 6 

                plt.subplot(241)
                plt.scatter(cloud_points[:, 0], cloud_points[:, 1], s=1, c="blue")
                plt.scatter(
                    filtered_cloud_points[:, 0],
                    filtered_cloud_points[:, 1],
                    s=1,
                    c="red",
                )
                plt.title("Input Points")

                plt.subplot(242)
                plt.imshow(
                    hist.T,
                    origin="lower",
                    extent=[
                        xedges[0] / 1000,
                        xedges[-1] / 1000,
                        yedges[0] / 1000,
                        yedges[-1] / 1000,
                    ],
                    aspect="auto",
                )
                plt.title("Histogram")

                plt.subplot(243)
                plt.imshow(
                    binary_map.T,
                    origin="lower",
                    extent=[
                        xedges[0] / 1000,
                        xedges[-1] / 1000,
                        yedges[0] / 1000,
                        yedges[-1] / 1000,
                    ],
                    aspect="auto",
                )
                plt.title("Binary Map")

                plt.subplot(244)
                plt.imshow(
                    final_map.T,
                    origin="lower",
                    extent=[
                        xedges[0] / 1000,
                        xedges[-1] / 1000,
                        yedges[0] / 1000,
                        yedges[-1] / 1000,
                    ],
                    cmap="plasma",
                    aspect="auto",
                )
                plt.plot(x_center, y_center, "g+", markersize=15)
                plt.title("Heatmap with Optimal Position")

                plt.subplot(245)
                plt.imshow(
                    heat_map.T,
                    origin="lower",
                    extent=[
                        xedges[0] / 1000,
                        xedges[-1] / 1000,
                        yedges[0] / 1000,
                        yedges[-1] / 1000,
                    ],
                    cmap="plasma",
                    aspect="auto",
                )
                plt.title("Heatmap")
                plt.colorbar()
                plt.subplot(246)
                plt.imshow(
                    cool_map.T,
                    origin="lower",
                    extent=[
                        xedges[0] / 1000,
                        xedges[-1] / 1000,
                        yedges[0] / 1000,
                        yedges[-1] / 1000,
                    ],
                    cmap="plasma",
                    aspect="auto",
                )
                plt.title("Coolmap")
                if closeness_map is not None:
                    plt.subplot(247)
                    plt.imshow(
                        closeness_map.T,
                        origin="lower",
                        extent=[
                            xedges[0] / 1000,
                            xedges[-1] / 1000,
                            yedges[0] / 1000,
                            yedges[-1] / 1000,
                        ],
                        cmap="plasma",
                        aspect="auto",
                    )
                    plt.title("Closeness Map")
                    plt.colorbar()

                if special_request_map is not None:
                    plt.subplot(248)
                    plt.imshow(
                        special_request_map.T,
                        origin="lower",
                        extent=[
                            xedges[0] / 1000,
                            xedges[-1] / 1000,
                            yedges[0] / 1000,
                            yedges[-1] / 1000,
                        ],
                        cmap="plasma",
                        aspect="auto",
                    )
                    plt.title(
                        f"{request_dict.get('request', 'Unknown')}, {request_dict.get('position', 'Unknown')}"
                    )

                output_dir = "/workspace/heatmap_results"
                os.makedirs(output_dir, exist_ok=True)
                plt.savefig(os.path.join(output_dir, "heatmap_result.png"))
                plt.close()
            except Exception as e:
                self.get_logger().error(f"Could not save image: {e}")

        return response

    def generate_close_to_heatmap(
        self,
        binary_map,
        close_point,
        min_x_mm=0,
        min_y_mm=0,
        grid_size_mm=10,
        max_distance=CLOSE_BY_MAX_DISTANCE,
    ):
        """
        Generate a heatmap based on proximity to a close point.
        """
        heatmap = np.zeros_like(binary_map, dtype=float)
        rows, cols = binary_map.shape
        for i in range(rows):
            for j in range(cols):
                if binary_map[i, j] == 1:
                    x = (min_x_mm + (i + 0.5) * grid_size_mm) / 1000.0
                    y = (min_y_mm + (j + 0.5) * grid_size_mm) / 1000.0
                    distance = np.linalg.norm(np.array([x, y]) - np.array(close_point))
                    if distance <= max_distance:
                        heatmap[i, j] = max(0, 1 - (distance / max_distance))
        return heatmap

    def generate_special_request_map(self, binary_map, request, request_dict):
        """
        Generate a special request map based on the request and request_dict.
        This is a placeholder function that can be customized based on specific requirements.
        """
        special_request_map = None
        if request_dict is not None:
            task = request_dict.get("request", "")
            position = request_dict.get("position", "")
            if task == "close_by":
                object_point = request.close_point

                if object_point.header.frame_id != "base_link":
                    self.get_logger().error(
                        f"Invalid close point, frame_id: {object_point.header.frame_id}"
                    )

                elif position == "close":
                    self.get_logger().info("Generating close to heatmap")
                    special_request_map = generate_close_to_heatmap(
                        binary_map,
                        [object_point.point.x, object_point.point.y],
                        self.min_x_mm,
                        self.min_y_mm,
                        self.grid_size_mm,
                    )
                elif position in ["front", "back", "left", "right"]:
                    self.get_logger().info("Generating directional heatmap")
                    special_request_map = generate_directional_heatmap(
                        binary_map,
                        [object_point.point.x, object_point.point.y],
                        position,
                        self.min_x_mm,
                        self.min_y_mm,
                        self.grid_size_mm,
                    )
                else:
                    self.get_logger().error(f"Unknown request: {position}")
            else:
                self.get_logger().error(f"Unknown task: {task}")

        else:
            self.get_logger().warn("No special request provided, returning binary map")

        return special_request_map


def main(args=None):
    rclpy.init(args=args)
    heatmap_service = HeatmapServer()
    rclpy.spin(heatmap_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
