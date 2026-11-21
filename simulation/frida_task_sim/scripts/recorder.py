#!/usr/bin/env python3
"""Records the overview camera to an mp4 with the clock, score, speech and events drawn on it."""

import json
import os
import subprocess

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from frida_task_sim import scenario as scenarios
from frida_task_sim.timeline import Timeline, clock

FPS = 4
FONT = cv2.FONT_HERSHEY_SIMPLEX


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder")
        scenario = scenarios.load(self.declare_parameter("scenario_file", "").value)
        run_dir = self.declare_parameter("run_dir", "").value
        self.path = os.path.join(run_dir, "overview.mp4")
        self.timeline = Timeline(scenario)
        self.title = f"{scenario['task'].upper()} seed {scenario['seed']}"
        self.t = 0.0
        self.speech = ""
        self.lines: list[tuple[str, bool]] = []
        self.bridge = CvBridge()
        self.ffmpeg = None
        self.create_subscription(String, "/sim/referee/events", self._on_event, 50)
        self.create_subscription(
            Image, "/sim/overview_camera/image", self._on_image, qos_profile_sensor_data
        )

    def _on_event(self, msg: String):
        event = json.loads(msg.data)
        self.t = event["t"]
        if event["type"] == "speech":
            self.speech = f"{'FRIDA' if event['who'] == 'robot' else event['who']}: {event['text']}"
        for _, text in self.timeline.add(event):
            if event["type"] not in ("speech", "step"):
                self.lines.append(
                    (text.strip(), "pts" in text and "-" in text.split("pts")[0])
                )
        self.lines = self.lines[-8:]

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8").copy()
        height, width = frame.shape[:2]
        if self.ffmpeg is None:
            self.ffmpeg = subprocess.Popen(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    "bgr24",
                    "-s",
                    f"{width}x{height}",
                    "-r",
                    str(FPS),
                    "-i",
                    "-",
                    "-pix_fmt",
                    "yuv420p",
                    "-vcodec",
                    "libx264",
                    "-preset",
                    "veryfast",
                    self.path,
                ],
                stdin=subprocess.PIPE,
            )
        cv2.rectangle(frame, (0, 0), (width, 44), (30, 30, 30), -1)
        header = f"{self.title}   {clock(self.t)}   SCORE {self.timeline.total}"
        cv2.putText(frame, header, (12, 31), FONT, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        for i, (text, bad) in enumerate(self.lines):
            colour = (80, 80, 255) if bad else (120, 255, 120)
            cv2.putText(
                frame, text, (12, 72 + 24 * i), FONT, 0.55, (0, 0, 0), 4, cv2.LINE_AA
            )
            cv2.putText(
                frame, text, (12, 72 + 24 * i), FONT, 0.55, colour, 1, cv2.LINE_AA
            )
        if self.speech:
            cv2.rectangle(frame, (0, height - 44), (width, height), (30, 30, 30), -1)
            cv2.putText(
                frame,
                self.speech[:95],
                (12, height - 14),
                FONT,
                0.65,
                (255, 230, 120),
                2,
                cv2.LINE_AA,
            )
        try:
            self.ffmpeg.stdin.write(frame.tobytes())
        except BrokenPipeError:
            pass

    def close(self):
        if self.ffmpeg is not None:
            self.ffmpeg.stdin.close()
            self.ffmpeg.wait(timeout=30)


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
