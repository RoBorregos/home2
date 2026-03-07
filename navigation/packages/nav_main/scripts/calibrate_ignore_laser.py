#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import argparse
import sys

class LidarCalibrator(Node):
    def __init__(self, max_radius, padding, num_scans):
        super().__init__('lidar_calibrator')
        self.max_radius = max_radius
        self.padding = padding
        self.num_scans_to_collect = num_scans
        self.scans_collected = []
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_input',
            self.listener_callback,
            10)
        
        self.get_logger().info(f"LiDAR Calibration Tool started.")
        self.get_logger().info(f"Waiting for {self.num_scans_to_collect} scans on /scan_input...")
        self.get_logger().info(f"Max robot radius: {self.max_radius}m, Padding: {self.padding} degrees.")

    def listener_callback(self, msg):
        self.scans_collected.append(msg)
        if len(self.scans_collected) >= self.num_scans_to_collect:
            self.process_scans()
            rclpy.shutdown()

    def rad2deg(self, rad):
        return rad * 180.0 / math.pi

    def process_scans(self):
        self.get_logger().info("Processing scans...")
        
        first_scan = self.scans_collected[0]
        num_readings = len(first_scan.ranges)
        
        all_ranges = np.array([scan.ranges for scan in self.scans_collected])

        all_ranges[all_ranges == 0.0] = 100.0
        all_ranges[np.isinf(all_ranges)] = 100.0
        
        min_ranges = np.min(all_ranges, axis=0)
        ignored_indices = np.where(min_ranges < self.max_radius)[0]
        
        if len(ignored_indices) == 0:
            self.get_logger().warn(f"No robot structural occlusions detected within {self.max_radius}m!")
            print(f"DEBUG: Closest point found was at {np.min(min_ranges):.3f}m")
            return

        segments = []
        if len(ignored_indices) > 0:
            start_idx = ignored_indices[0]
            for i in range(1, len(ignored_indices)):
                if ignored_indices[i] != ignored_indices[i-1] + 1:
                    segments.append((start_idx, ignored_indices[i-1]))
                    start_idx = ignored_indices[i]
            segments.append((start_idx, ignored_indices[-1]))

        intervals = []
        for start_idx, end_idx in segments:
            angle_start = self.rad2deg(first_scan.angle_min + start_idx * first_scan.angle_increment)
            angle_end = self.rad2deg(first_scan.angle_min + end_idx * first_scan.angle_increment)
            intervals.append([angle_start - self.padding, angle_end + self.padding])

        intervals.sort()
        merged = []
        if intervals:
            curr_start, curr_end = intervals[0]
            for i in range(1, len(intervals)):
                next_start, next_end = intervals[i]
                if next_start <= curr_end + 1.0: # Merge if they are within 1 degree
                    curr_end = max(curr_end, next_end)
                else:
                    merged.append([curr_start, curr_end])
                    curr_start, curr_end = next_start, next_end
            merged.append([curr_start, curr_end])

        normalized_intervals = []
        for start, end in merged:
            if start < -180:
                normalized_intervals.append([-180, end])
                normalized_intervals.append([start + 360, 180])
            elif end > 180:
                normalized_intervals.append([start, 180])
                normalized_intervals.append([-180, end - 360])
            else:
                normalized_intervals.append([start, end])

        normalized_intervals.sort()
        consolidated = []
        if normalized_intervals:
            curr_start, curr_end = normalized_intervals[0]
            for i in range(1, len(normalized_intervals)):
                next_start, next_end = normalized_intervals[i]
                if next_start <= curr_end + 1.0:
                    curr_end = max(curr_end, next_end)
                else:
                    consolidated.append([curr_start, curr_end])
                    curr_start, curr_end = next_start, next_end
            consolidated.append([curr_start, curr_end])

        output_parts = []
        for start, end in consolidated:
            output_parts.append(f"{start:.1f}, {end:.1f}")
        
        ignore_array_str = ", ".join(output_parts)
        
        print("\n" + "="*50)
        print("CALIBRATION COMPLETE (Safe Mode)")
        print("="*50)
        print(f"Detected {len(consolidated)} consolidated occlusion segments.")
        print(f"Max Radius: {self.max_radius}m, Padding: {self.padding} deg")
        print("\nRecommended ignore_array (copy and paste between the ''):")
        print(f"{ignore_array_str}")
        print("\n" + "="*50 + "\n")

def main():
    parser = argparse.ArgumentParser(description='LiDAR Calibration Tool for ignore_laser node')
    parser.add_argument('--max-radius', type=float, default=0.3, help='Max distance to consider as robot structure (meters)')
    parser.add_argument('--padding', type=float, default=2.0, help='Degrees of padding to add to each side of occlusions')
    parser.add_argument('--scans', type=int, default=50, help='Number of scans to average for calibration')
    
    ros_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    args, unknown = parser.parse_known_args(ros_args)

    rclpy.init()
    node = LidarCalibrator(args.max_radius, args.padding, args.scans)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
