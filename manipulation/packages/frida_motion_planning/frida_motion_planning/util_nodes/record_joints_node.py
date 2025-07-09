#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from sensor_msgs.msg import JointState
from threading import Lock


class JointRecorder(Node):
    def __init__(self, file_path, frequency, joint_names):
        super().__init__("joint_recorder")
        callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        file_param = self.declare_parameter("file_path", file_path)
        self.file_path = file_param.value
        frequency_param = self.declare_parameter("frequency", frequency)
        self.frequency = frequency_param.value

        self.frequency = frequency
        self.joint_names = joint_names
        self.joint_values = {}
        self.lock = Lock()

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos,
            callback_group=callback_group,
        )

        self.csv_file = open(self.file_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time"] + self.joint_names)

        self.timer = self.create_timer(1.0 / self.frequency, self.recording_callback)

    def joint_state_callback(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.joint_names:
                    self.joint_values[name] = position

    def recording_callback(self):
        with self.lock:
            current_time = (
                self.get_clock().now().seconds_nanoseconds()[0]
                + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
            )
            row = [current_time] + [
                self.joint_values.get(name, None) for name in self.joint_names
            ]
            self.get_logger().info(f"Recording joint data: {row}")
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def destroy_node(self):
        self.get_logger().info(f"Saving joint data to {self.file_path}")
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    file_path = "joint_data.csv"
    frequency = 10  # Frequency in Hz
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    recorder = JointRecorder(file_path, frequency, joint_names)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down joint recorder...")
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
