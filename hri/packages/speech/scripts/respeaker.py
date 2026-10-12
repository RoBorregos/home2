#!/usr/bin/env python3

import math
from collections import deque

import rclpy
import usb.core
import usb.util
from pixel_ring import pixel_ring
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.tuning import Tuning
from std_msgs.msg import Int16, String


class AngleMovingAverage:
    """Moving average of angles in degrees, wrap-aware.

    Averages the unit vectors instead of the raw degrees: a plain mean of 350
    and 10 gives 180 (the opposite side) instead of 0.
    """

    def __init__(self, window_size):
        self.angles = deque(maxlen=window_size)

    def next(self, degrees):
        self.angles.append(math.radians(degrees))
        x = sum(math.cos(a) for a in self.angles)
        y = sum(math.sin(a) for a in self.angles)
        return math.degrees(math.atan2(y, x)) % 360.0


class Respeaker(Node):
    def __init__(self):
        super().__init__("respeaker")

        # Ros parameters
        self.declare_parameter("RESPEAKER_DOA_TOPIC", "/respeaker/doa")
        self.declare_parameter("doa_timer", 0.5)  # seconds
        self.declare_parameter("RESPEAKER_LIGHT_TOPIC", "/respeaker/light")

        doa_publish_topic = (
            self.get_parameter("RESPEAKER_DOA_TOPIC").get_parameter_value().string_value
        )
        doa_timer = self.get_parameter("doa_timer").get_parameter_value().double_value
        light_subscriber_topic = (
            self.get_parameter("RESPEAKER_LIGHT_TOPIC")
            .get_parameter_value()
            .string_value
        )

        # Properties
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.moving_average = AngleMovingAverage(10)

        if self.dev:
            self.tuning = Tuning(self.dev)
        else:
            self.tuning = None
            self.get_logger().error("Respeaker not found.")

        # Ros interactions
        self.publisher_ = self.create_publisher(Int16, doa_publish_topic, 20)
        self.create_timer(doa_timer, self.publish_DOA)
        self.create_subscription(
            String, light_subscriber_topic, self.callback_light, 10
        )

        self.get_logger().info("Respeaker ready")

    def publish_DOA(self):
        if self.tuning:
            next_angle = self.moving_average.next(self.tuning.direction)
            self.publisher_.publish(Int16(data=int(round(next_angle)) % 360))
        else:
            self.get_logger().error("Respeaker not found.")

    def callback_light(self, data):
        command = data.data

        if command == "off":
            pixel_ring.off()
        elif command == "think" or command == "loading":
            pixel_ring.think()
        elif command == "speak":
            pixel_ring.speak()
        elif command == "listen":
            pixel_ring.listen()
        else:
            self.get_logger().warn("Command: " + command + " not supported")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Respeaker())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
