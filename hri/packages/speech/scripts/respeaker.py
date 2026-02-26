#!/usr/bin/env python3

import rclpy
import usb.core
import usb.util
from pixel_ring import pixel_ring
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.tuning import Tuning
from std_msgs.msg import Int16, String


class MovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = [0] * window_size  # Circular buffer
        self.sum = 0
        self.size = 0
        self.index = 0

    def next(self, val):
        if self.size < self.window_size:
            self.size += 1

        self.sum -= self.data[self.index]
        self.data[self.index] = val
        self.sum += val
        self.index = (self.index + 1) % self.window_size

        return self.sum / self.size


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
        self.moving_average = MovingAverage(10)

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

        self.get_logger().info("Respeaker node initialized.")

    def publish_DOA(self):
        if self.tuning:
            next_angle = self.moving_average.next(self.tuning.direction)
            self.publisher_.publish(Int16(data=int(next_angle)))
        else:
            self.get_logger().error("Respeaker not found.")

    def callback_light(self, data):
        command = data.data

<<<<<<< HEAD
        if command == "off" or command == "think" or command == "speak":
            pixel_ring.off()
=======
        if command == "off":
            pixel_ring.off()
        elif command == "think" or command == "loading":
            pixel_ring.think()
        elif command == "speak":
            pixel_ring.speak()
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
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
