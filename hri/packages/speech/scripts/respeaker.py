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

        #DSP
        self.declare_parameter("dsp.agc.enable", True)
        self.declare_parameter("dsp.agc.desired_level_dbov", -23.0)   # AGCDESIREDLEVEL
        self.declare_parameter("dsp.agc.max_gain_factor", 31.6)       # ~30 dB since 20*log10(31.6)≈30
        self.declare_parameter("dsp.agc.time_constant_s", 0.3)  

        # Noise suppression
        self.declare_parameter("dsp.ns.stationary_on", True)          # STATNOISEONOFF
        self.declare_parameter("dsp.ns.nonstationary_on", True)       # NONSTATNOISEONOFF

        # High-pass filter
        self.declare_parameter("dsp.hpf.mode", 2)                     # HPFONOFF: 0=OFF,1=70Hz,2=125Hz,3=180Hz

        # Echo suppression / AEC
        self.declare_parameter("dsp.aec.enable", True)                # ECHOONOFF + unfreeze AEC
        self.declare_parameter("dsp.aec.gamma_e", 1.0)                # GAMMA_E (direct/early)
        self.declare_parameter("dsp.aec.gamma_etail", 1.0)            # GAMMA_ETAIL (tail)
        self.declare_parameter("dsp.aec.gamma_enl", 1.0) 

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
            self.dsp_setting()
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

        if command == "off" or command == "think" or command == "speak":
            pixel_ring.off()
        elif command == "listen":
            pixel_ring.listen()
        else:
            self.get_logger().warn("Command: " + command + " not supported")
    
    def dsp_setting(self):
        # To good functionalty for the AEC, audio should be streamed to the speaker and the respeaker.
        try:
            # AGC
            agc_on = self.get_parameter("dsp.agc.enable").value
            self.tuning.write("AGCONOFF", 1 if agc_on else 0)
            self.tuning.write("AGCDESIREDLEVEL", float(self.get_parameter("dsp.agc.desired_level_dbov").value))
            self.tuning.write("AGCMAXGAIN", float(self.get_parameter("dsp.agc.max_gain_factor").value))  # factor g, where dB = 20*log10(g)
            self.tuning.write("AGCTIME", float(self.get_parameter("dsp.agc.time_constant_s").value))

            # Noise suppression
            self.tuning.write("STATNOISEONOFF", 1 if self.get_parameter("dsp.ns.stationary_on").value else 0)
            self.tuning.write("NONSTATNOISEONOFF", 1 if self.get_parameter("dsp.ns.nonstationary_on").value else 0)

            # High-pass filter
            self.tuning.write("HPFONOFF", int(self.get_parameter("dsp.hpf.mode").value))

            # Echo suppression / AEC
            aec_on = self.get_parameter("dsp.aec.enable").value
            self.tuning.write("ECHOONOFF", 1 if aec_on else 0)
            # Unfreeze AEC adaptation if enabled
            self.tuning.write("AECFREEZEONOFF", 0 if aec_on else 1)
            self.tuning.write("GAMMA_E", float(self.get_parameter("dsp.aec.gamma_e").value))
            self.tuning.write("GAMMA_ETAIL", float(self.get_parameter("dsp.aec.gamma_etail").value))
            self.tuning.write("GAMMA_ENL", float(self.get_parameter("dsp.aec.gamma_enl").value))

            self.get_logger().info("Applied ReSpeaker DSP settings.")
        except Exception as e:
            self.get_logger().error(f"Failed to apply DSP settings: {e}")


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
