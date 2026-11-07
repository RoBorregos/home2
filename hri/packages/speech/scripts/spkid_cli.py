#!/usr/bin/env python3
"""Command-line client for the speaker_verification node.

ros2 run speech spkid_cli.py enroll mario --seconds 15   # talk during 15 s
ros2 run speech spkid_cli.py lookback mario --seconds 20 # enroll from the last 20 s
ros2 run speech spkid_cli.py target mario                # "" -> any enrolled
ros2 run speech spkid_cli.py list | forget mario | abort | reload
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from frida_constants.hri_constants import (
    SPEAKER_ID_COMMAND_TOPIC,
    SPEAKER_ID_STATUS_TOPIC,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Cli(Node):
    def __init__(self) -> None:
        super().__init__("spkid_cli")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(String, SPEAKER_ID_COMMAND_TOPIC, 10)
        self.create_subscription(String, SPEAKER_ID_STATUS_TOPIC, self._cb, qos)
        self.last: dict | None = None
        self.t_sent = 0.0

    def _cb(self, msg: String) -> None:
        data = json.loads(msg.data)
        if data.get("t", 0) < self.t_sent:
            return  # latched message from before our command
        self.last = data
        if data["op"] == "enroll_progress":
            print(f"  ... {data['message']}")

    def send(self, payload: dict) -> None:
        # wait for the node's subscription so the first message is not lost
        deadline = time.time() + 5.0
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            sys.exit("speaker_verification node not found (is it running?)")
        self.t_sent = time.time() - 0.05
        self.last = None
        self.pub.publish(String(data=json.dumps(payload)))

    def wait_reply(self, ops: tuple[str, ...], timeout: float = 10.0) -> dict | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last and self.last["op"] in ops:
                return self.last
        return None

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def show(reply: dict | None) -> None:
    if reply is None:
        print("no reply from speaker_verification (timeout)")
        return
    mark = "OK " if reply["ok"] else "ERR"
    print(f"[{mark}] {reply['op']}: {reply['message']}")
    print(f"      target: {reply['target'] or 'any enrolled'}")
    for prof in reply.get("profiles", []):
        print(
            f"      - {prof['name']:<12} {prof['enrolled_sec']:>5.1f}s "
            f"consistency={prof['consistency']:.2f} pool={prof['pool']} "
            f"({prof['created']})"
        )


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    en = sub.add_parser("enroll", help="live enrollment session")
    en.add_argument("name")
    en.add_argument("--seconds", type=float, default=15.0)
    lb = sub.add_parser("lookback", help="enroll from recent speech")
    lb.add_argument("name")
    lb.add_argument("--seconds", type=float, default=20.0)
    tg = sub.add_parser("target")
    tg.add_argument("name", nargs="?", default="")
    fg = sub.add_parser("forget")
    fg.add_argument("name")
    sub.add_parser("list")
    sub.add_parser("abort")
    sub.add_parser("reload")
    args = ap.parse_args()

    rclpy.init()
    cli = Cli()
    try:
        if args.cmd == "enroll":
            cli.send({"op": "enroll_start"})
            show(cli.wait_reply(("enroll_start",)))
            print(
                f"\n>>> Habla ahora durante {args.seconds:.0f} s (frases normales, pausas cortas)\n"
            )
            end = time.time() + args.seconds
            while time.time() < end:
                cli.spin_for(1.0)
                left = end - time.time()
                if left > 0:
                    print(f"  {left:4.0f} s restantes", end="\r", flush=True)
            print("\n>>> Listo, deja de hablar. Esperando el último segmento...")
            cli.spin_for(1.5)  # let the final segment close (MIN_SILENCE_SEC)
            cli.send({"op": "enroll_commit", "name": args.name})
            show(cli.wait_reply(("enroll_commit",), timeout=20.0))
        elif args.cmd == "lookback":
            cli.send(
                {"op": "enroll_lookback", "name": args.name, "seconds": args.seconds}
            )
            show(cli.wait_reply(("enroll_commit",), timeout=20.0))
        elif args.cmd == "target":
            cli.send({"op": "set_target", "name": args.name})
            show(cli.wait_reply(("set_target",)))
        elif args.cmd == "forget":
            cli.send({"op": "forget", "name": args.name})
            show(cli.wait_reply(("forget",)))
        elif args.cmd == "abort":
            cli.send({"op": "enroll_abort"})
            show(cli.wait_reply(("enroll_abort",)))
        elif args.cmd == "reload":
            cli.send({"op": "reload"})
            show(cli.wait_reply(("reload",)))
        else:
            cli.send({"op": "list"})
            show(cli.wait_reply(("list",)))
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
