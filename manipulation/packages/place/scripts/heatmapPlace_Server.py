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

# change matplotlib backend to avoid X server dependency
import matplotlib

matplotlib.use("Agg")  # Use a non-interactive backend


class HeatmapServer(Node):
    def __init__(self):
        super().__init__("heatmap_service")
        self.save_image = False
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

        grid_size = 0.015  # meters
        grid_size_mm = int(grid_size * 1000)

        # Convert points to mm coordinates
        x_mm = (cloud_points[:, 0] * 1000).astype(int)
        y_mm = (cloud_points[:, 1] * 1000).astype(int)

        # Create histogram bins
        x_range = np.arange(min(x_mm), max(x_mm) + grid_size_mm, grid_size_mm)
        y_range = np.arange(min(y_mm), max(y_mm) + grid_size_mm, grid_size_mm)
        hist, xedges, yedges = np.histogram2d(x_mm, y_mm, bins=[x_range, y_range])
        min_x_mm = xedges[0]
        min_y_mm = yedges[0]

        # Generate binary map
        binary_map = hist > 0

        # Create heat/cool kernels
        heat_kernel_length = 0.3  # meters
        cool_kernel_length = 0.2  # meters
        heat_multiplier = 1.0
        cool_multiplier = 10.0

        # Heat kernel
        heat_kernel_size = int(heat_kernel_length / grid_size)
        xx, yy = np.mgrid[
            -heat_kernel_size // 2 : heat_kernel_size // 2 + 1,
            -heat_kernel_size // 2 : heat_kernel_size // 2 + 1,
        ]
        heat_kernel = np.exp(-(xx**2 + yy**2) / (2 * (heat_kernel_size / 2) ** 2))
        heat_kernel *= heat_multiplier

        # Cool kernel
        cool_kernel_size = int(cool_kernel_length / grid_size)
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
                    x = (min_x_mm + (i + 0.5) * grid_size_mm) / 1000.0
                    y = (min_y_mm + (j + 0.5) * grid_size_mm) / 1000.0
                    closeness_map[i, j] = (np.sqrt(x**2 + y**2)) ** 2
            closeness_map = np.clip(closeness_map, 0, None)
            closeness_map = np.exp(-closeness_map / (grid_size_mm * 2)) ** 2

            final_map = final_map * closeness_map

            # Normalize final map
            final_map = final_map / np.max(final_map)
            final_map = np.clip(final_map, 0, None)

        # Find best position
        max_idx = np.unravel_index(np.argmax(final_map), final_map.shape)

        x_center = (min_x_mm + (max_idx[0] + 0.5) * grid_size_mm) / 1000.0
        y_center = (min_y_mm + (max_idx[1] + 0.5) * grid_size_mm) / 1000.0

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
            plt.figure(figsize=(16, 6))

            plt.subplot(241)
            plt.scatter(cloud_points[:, 0], cloud_points[:, 1], s=1, c="blue")
            plt.scatter(
                filtered_cloud_points[:, 0], filtered_cloud_points[:, 1], s=1, c="red"
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
            output_dir = "/workspace/heatmap_results"
            os.makedirs(output_dir, exist_ok=True)
            plt.savefig(os.path.join(output_dir, "heatmap_result.png"))
            plt.close()

        return response


def main(args=None):
    rclpy.init(args=args)
    heatmap_service = HeatmapServer()
    rclpy.spin(heatmap_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
