#!/usr/bin/env python3
"""Relay mínimo PointCloud2 (suple a topic_tools, ausente en el contenedor).
Republica la nube de la ZED como /point_cloud para alimentar el octomap de MoveIt.
Uso: python3 cloud_relay.py [--in TOPIC] [--out TOPIC]"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="inp", default="/zed/zed_node/point_cloud/cloud_registered")
    p.add_argument("--out", dest="out", default="/point_cloud")
    a = p.parse_args()
    rclpy.init()
    n = Node("cloud_relay")
    pub = n.create_publisher(PointCloud2, a.out, qos_profile_sensor_data)
    n.create_subscription(PointCloud2, a.inp, lambda m: pub.publish(m), qos_profile_sensor_data)
    n.get_logger().info(f"relay {a.inp} -> {a.out}")
    rclpy.spin(n)


if __name__ == "__main__":
    main()
