#!/usr/bin/env python3
"""Handle wakeword events and DOA to choose which mic/channel to keep open.

Subscribes to:
- wakeword topic (default `/speech/oww`) publishing `std_msgs/String` with a dict-like string
- Respeaker DOA topic (default `/respeaker/doa`) publishing `std_msgs/Int16` angle

Publishes to:
- `/speech/active_mic` (std_msgs/String) with JSON like {"keyword":"frida","doa":45,"channel":1}

This node is intentionally lightweight and only decides the channel to prefer.
Actual audio capture adapts by subscribing to `/speech/active_mic`.
"""

import ast
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16


class KwsDoaHandler(Node):
    def __init__(self):
        super().__init__("kws_doa_handler")

        self.declare_parameter("WAKEWORD_TOPIC", "/speech/oww")
        self.declare_parameter("DOA_TOPIC", "/respeaker/doa")
        self.declare_parameter("ACTIVE_MIC_TOPIC", "/speech/active_mic")
        self.declare_parameter("MIC_COUNT", 4)

        wake_topic = (
            self.get_parameter("WAKEWORD_TOPIC").get_parameter_value().string_value
        )
        doa_topic = self.get_parameter("DOA_TOPIC").get_parameter_value().string_value
        self.active_mic_topic = (
            self.get_parameter("ACTIVE_MIC_TOPIC").get_parameter_value().string_value
        )
        self.mic_count = (
            self.get_parameter("MIC_COUNT").get_parameter_value().integer_value
        )

        self.last_doa = None

        self.create_subscription(Int16, doa_topic, self._doa_cb, 10)
        self.create_subscription(String, wake_topic, self._wake_cb, 10)

        self.publisher = self.create_publisher(String, self.active_mic_topic, 10)

        self.get_logger().info(
            f"KwsDoaHandler initialized: wake={wake_topic}, doa={doa_topic}, publish={self.active_mic_topic}"
        )

    def _doa_cb(self, msg: Int16):
        try:
            angle = int(msg.data)
            # normalize
            angle = angle % 360
            self.last_doa = angle
        except Exception as e:
            self.get_logger().warn(f"Invalid DOA message: {e}")

    def _wake_cb(self, msg: String):
        payload = msg.data
        keyword = None
        score = None

        # try to parse payload: it was published in kws_oww as str(detection_info)
        try:
            info = ast.literal_eval(payload)
            if isinstance(info, dict):
                keyword = info.get("keyword")
                score = info.get("score")
        except Exception:
            # fallback: treat payload as plain keyword
            keyword = payload

        doa = self.last_doa if self.last_doa is not None else -1

        channel = self.map_doa_to_channel(doa)

        out = {"keyword": keyword, "score": score, "doa": doa, "channel": channel}
        s = json.dumps(out)

        self.get_logger().info(
            f"Wakeword detected -> doa={doa}, selecting channel={channel}"
        )
        self.publisher.publish(String(data=s))

    def map_doa_to_channel(self, angle: int) -> int:
        """Map DOA angle (0-359) to mic channel index (0..mic_count-1).

        Uses equal angular sectors, rotated so channel 0 is centered at angle 0.
        """
        try:
            if angle < 0:
                return 0
            sector = 360.0 / float(max(1, self.mic_count))
            # shift angle so sector 0 is centered at 0
            half = sector / 2.0
            a = (angle + half) % 360.0
            ch = int(math.floor(a / sector)) % self.mic_count
            return ch
        except Exception:
            return 0


def main(args=None):
    rclpy.init(args=args)
    node = KwsDoaHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
