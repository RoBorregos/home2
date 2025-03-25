#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import os
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from frida_interfaces.srv import HeatmapPlace
from scipy.signal import convolve2d


class HeatmapServer(Node):
    def __init__(self):
        super().__init__("heatmap_service")
        self.declare_parameter("SAVE_IMAGE", True)
        self.save_image = (
            self.get_parameter("SAVE_IMAGE").get_parameter_value().bool_value
        )

        self.srv = self.create_service(
            HeatmapPlace, "HeatmapPlace", self.handle_heatmap
        )
        self.get_logger().info("Heatmap Place Service ready")

    def handle_heatmap(self, request, response):
        # Convert PointCloud2 to numpy array
        point_cloud = request.pointcloud
        cloud_gen = point_cloud2.read_points(
            point_cloud, field_names=("x", "y", "z"), skip_nans=True
        )

        cloud_points = []
        for p in cloud_gen:
            cloud_points.append([p[0], p[1]])
        cloud_points = np.array(cloud_points)

        # If no points found, return zero point
        if len(cloud_points) == 0:
            self.get_logger().warn("Empty pointcloud received")
            response.point = Point()
            return response

        grid_size = 0.02  # meters
        grid_size_mm = int(grid_size * 1000)

        # Convert points to mm coordinates
        x_mm = (cloud_points[:, 0] * 1000).astype(int)
        y_mm = (cloud_points[:, 1] * 1000).astype(int)

        # Create histogram bins
        x_range = np.arange(min(x_mm), max(x_mm) + grid_size_mm, grid_size_mm)
        y_range = np.arange(min(y_mm), max(y_mm) + grid_size_mm, grid_size_mm)
        hist, xedges, yedges = np.histogram2d(x_mm, y_mm, bins=[x_range, y_range])

        # Generate binary map
        binary_map = hist > 0

        # Create heat/cool kernels
        heat_kernel_length = 0.4  # meters
        cool_kernel_length = 0.3  # meters
        heat_multiplier = 1.0
        cool_multiplier = 1.5

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

        # Apply convolution

        heat_map = convolve2d(binary_map.astype(float), heat_kernel, mode="same")
        cool_map = convolve2d((~binary_map).astype(float), cool_kernel, mode="same")

        # Combine maps
        final_map = heat_map - cool_map
        final_map[~binary_map] = 0
        final_map = np.clip(final_map, 0, None)

        # Find best position
        max_idx = np.unravel_index(np.argmax(final_map), final_map.shape)
        min_x_mm = xedges[0]
        min_y_mm = yedges[0]

        x_center = (min_x_mm + (max_idx[0] + 0.5) * grid_size_mm) / 1000.0
        y_center = (min_y_mm + (max_idx[1] + 0.5) * grid_size_mm) / 1000.0

        # Create response
        response.point = Point()
        response.point.x = float(x_center)
        response.point.y = float(y_center)
        response.point.z = 0.0  # Assuming planar surface

        # Save visualization
        if self.save_image:
            plt.figure(figsize=(12, 8))

            plt.subplot(221)
            plt.scatter(cloud_points[:, 0], cloud_points[:, 1], s=1)
            plt.title("Input Points")

            plt.subplot(222)
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

            plt.subplot(223)
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

            plt.subplot(224)
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

            output_dir = os.path.join(os.getcwd(), "heatmap_images")
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
