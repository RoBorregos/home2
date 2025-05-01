#!/usr/bin/env python3

import json

import rclpy
import usb.core
import usb.util
from pixel_ring import pixel_ring
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.tuning import PARAMETERS, Tuning
from std_msgs.msg import Int16, String

from frida_interfaces.srv import SetRespeakerParams


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
        self.declare_parameter("RESPEAKER_PARAMS_SERVICE", "/respeaker/params")

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
        RESPEAKER_PARAMS_SERVICE = (
            self.get_parameter("RESPEAKER_PARAMS_SERVICE")
            .get_parameter_value()
            .string_value
        )
        self.srv = self.create_service(
            SetRespeakerParams, RESPEAKER_PARAMS_SERVICE, self.config_params
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

        if command == "off" or command == "think" or command == "speak":
            pixel_ring.off()
        elif command == "listen":
            pixel_ring.listen()
        else:
            self.get_logger().warn("Command: " + command + " not supported")

    def save_config(self, filename="config_backup.json"):
        if not self.dev:
            self.get_logger().info("No device found")
            return

        config = {}
        for name in sorted(PARAMETERS.keys()):
            if (
                PARAMETERS[name][5] != "ro"
            ):  # skip read-only if you want to save only rw
                try:
                    value = self.dev.read(name)
                    config[name] = value
                except Exception as e:
                    self.get_logger().info(f"Failed to read {name}: {e}")

        with open(filename, "w") as f:
            json.dump(config, f, indent=2)

        self.get_logger().info(f"Configuration saved to {filename}")

    def config_params(
        self, request: SetRespeakerParams.Request, response: SetRespeakerParams.Response
    ):
        try:
            if request.action == "save":
                self.save_config()
                response.success = True
            elif request.action == "load":
                self.load_config()
                response.success = True
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False

        return response

    def load_config(self, filename="config_backup.json"):
        if not self.dev:
            self.get_logger().info("No device found")
            return

        with open(filename, "r") as f:
            config = json.load(f)

        for name, value in config.items():
            if name in PARAMETERS and PARAMETERS[name][5] != "ro":
                try:
                    self.dev.write(name, value)
                    self.get_logger().info(f"Set {name} = {value}")
                except Exception as e:
                    self.get_logger().info(f"Failed to write {name}: {e}")
            else:
                self.get_logger().info(f"Skipping {name}: Not writable or unknown")


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
