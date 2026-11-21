#!/usr/bin/env python3
"""Live view of who is speaking to the robot and from where.

    ros2 run speech who_where_monitor.py

Prints one line per speaker-id decision (partial windows dimmed, finals bold)
from SPEAKER_ID_LOCATED_TOPIC, plus a compass with the live direction from
SOUND_DOA_TOPIC while someone is speaking.
"""

import json
import time

import rclpy
from frida_constants.hri_constants import SOUND_DOA_TOPIC, SPEAKER_ID_LOCATED_TOPIC
from rclpy.node import Node
from std_msgs.msg import String

BOLD, DIM, GREEN, YELLOW, RESET = (
    "\033[1m",
    "\033[2m",
    "\033[32m",
    "\033[33m",
    "\033[0m",
)
ARROWS = {
    "front": "↑",
    "front-left": "↖",
    "left": "←",
    "back-left": "↙",
    "back": "↓",
    "back-right": "↘",
    "right": "→",
    "front-right": "↗",
}


class WhoWhereMonitor(Node):
    def __init__(self):
        super().__init__("who_where_monitor")
        self.create_subscription(String, SPEAKER_ID_LOCATED_TOPIC, self._located, 10)
        self.create_subscription(String, SOUND_DOA_TOPIC, self._doa, 10)
        self._last_live = 0.0
        print(f"{BOLD}who / where monitor{RESET}  (0 deg = front, +90 left, -90 right)")

    def _located(self, msg):
        d = json.loads(msg.data)
        who = d["speaker"]
        color = GREEN if who != "unknown" else YELLOW
        style = BOLD if d["final"] else DIM
        kind = "FINAL  " if d["final"] else "partial"
        srcs = d.get("sources", [])
        extra = (
            f"  (+ another voice at {srcs[1]['azimuth_deg']:+.0f}deg)"
            if len(srcs) > 1
            else ""
        )
        rel = "" if d["doa_reliable"] else " (low conf)"
        via = " [by direction]" if d.get("via") == "direction" else ""
        print(
            f"{style}{time.strftime('%H:%M:%S')} {kind} seg{d['seg_id']:<4} "
            f"{color}{who.upper():>8}{RESET}{style} best={d['best']}:{d['score']:.2f} "
            f"{ARROWS.get(d['direction'], '?')} {d['direction']:<11} "
            f"{d['azimuth_deg']:+5.0f}deg conf={d['doa_confidence']:.2f}{rel} "
            f"{d['dur']:.1f}s{via}{extra}{RESET}",
            flush=True,
        )

    def _doa(self, msg):
        d = json.loads(msg.data)
        now = time.time()
        if now - self._last_live < 1.0:
            return
        self._last_live = now
        print(
            f"{DIM}         live sound   {ARROWS.get(d['direction'], '?')} "
            f"{d['direction']:<11} {d['azimuth_deg']:+5.0f}deg conf={d['confidence']:.2f}{RESET}",
            flush=True,
        )


def main():
    rclpy.init()
    node = WhoWhereMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
