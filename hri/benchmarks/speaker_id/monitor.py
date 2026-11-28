#!/usr/bin/env python3
"""Live tool for speaker_verification (needs the HRI pipeline running).

python3 monitor.py watch                       # who is speaking and from where
python3 monitor.py enroll mario --seconds 20   # talk while it records
python3 monitor.py lookback mario --seconds 20 # enroll from the last 20 s
python3 monitor.py list | forget mario | target mario
python3 monitor.py session --positions front,right,left
    # guided live test: ENTER at each position, then speak; prints a table
"""

import argparse
import json
import sys
import time
from collections import Counter

import numpy as np
import rclpy
from frida_constants.hri_constants import (
    SPEAKER_ID_COMMAND_TOPIC,
    SPEAKER_ID_LOCATED_TOPIC,
    SPEAKER_ID_STATUS_TOPIC,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

BOLD, DIM, GREEN, YELLOW, RESET = (
    "\033[1m",
    "\033[2m",
    "\033[32m",
    "\033[33m",
    "\033[0m",
)
EXPECTED_DEG = {"front": 0, "left": 90, "right": -90, "back": 180}


def circ_mean(deg):
    return float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(deg))))))


class Monitor:
    def __init__(self):
        self.node = rclpy.create_node("speaker_id_monitor")
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cmd_pub = self.node.create_publisher(String, SPEAKER_ID_COMMAND_TOPIC, 10)
        self.node.create_subscription(
            String, SPEAKER_ID_STATUS_TOPIC, self._status, latched
        )
        self.node.create_subscription(
            String, SPEAKER_ID_LOCATED_TOPIC, self._located, 50
        )
        self.located, self.reply, self.t_sent, self.verbose = [], None, 0.0, False

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # ------------------------------------------------------------ located
    def _located(self, msg):
        d = json.loads(msg.data)
        self.located.append(d)
        if not self.verbose:
            return
        who = d["speaker"]
        az = d["azimuth_deg"]
        where = f"{d['direction']:<11} {az:+5.0f}deg" if az is not None else "no DOA"
        print(
            f"{BOLD if d['final'] else DIM}{time.strftime('%H:%M:%S')} "
            f"{'FINAL  ' if d['final'] else 'partial'} seg{d['seg_id']:<4} "
            f"{GREEN if who != 'unknown' else YELLOW}{who.upper():>8}{RESET} "
            f"{d['label']:<10} score={d['score']:.2f} {where} "
            f"conf={d['doa_confidence']:.2f} {d['dur']:.1f}s"
            f"{' [by direction]' if d['via'] == 'direction' else ''}"
            f"{' [' + d['part'] + ' voice]' if d['part'] != 'whole' else ''}{RESET}",
            flush=True,
        )

    # ----------------------------------------------------------- commands
    def _status(self, msg):
        d = json.loads(msg.data)
        if d.get("t", 0) < self.t_sent:
            return  # latched reply from an earlier command
        self.reply = d
        if d["op"] == "enroll_progress":
            print(f"  ... {d['message']}")

    def command(self, payload, reply_op, timeout=10.0):
        deadline = time.time() + 5.0
        while self.cmd_pub.get_subscription_count() == 0 and time.time() < deadline:
            self.spin(0.1)
        if self.cmd_pub.get_subscription_count() == 0:
            sys.exit("speaker_verification is not running")
        self.t_sent, self.reply = time.time() - 0.05, None
        self.cmd_pub.publish(String(data=json.dumps(payload)))
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.spin(0.1)
            if self.reply and self.reply["op"] == reply_op:
                break
        r = self.reply
        if r is None or r["op"] != reply_op:
            print("no reply from speaker_verification (timeout)")
            return
        print(f"[{'OK ' if r['ok'] else 'ERR'}] {r['op']}: {r['message']}")
        print(f"      target: {r['target'] or 'any enrolled'}")
        for p in r.get("profiles", []):
            print(
                f"      - {p['name']:<12} {p['enrolled_sec']:>5.1f}s "
                f"consistency={p['consistency']:.2f} pool={p['pool']}"
            )

    # ------------------------------------------------------------ session
    def session(self, positions, secs, grace):
        rows = []
        for pos in positions:
            input(
                f"\n{BOLD}>>> Stand at the robot's {pos.upper()} (~1 m), press ENTER{RESET}"
            )
            self.spin(1.0)
            self.located.clear()
            print(f"{BOLD}>>> SPEAK NOW for {secs:.0f} s{RESET}")
            self.spin(secs)
            # segments still open when the window closes publish their final later
            seen = {m["seg_id"] for m in self.located}
            end = time.time() + grace
            while time.time() < end and not seen <= {
                m["seg_id"] for m in self.located if m["final"]
            }:
                self.spin(0.2)
            finals = [m for m in self.located if m["final"] and m["seg_id"] in seen]
            finals = finals or self.located
            named = [m for m in finals if m["speaker"] != "unknown"]
            finals = named or finals  # MIXED segments are split per voice
            if not finals:
                rows.append((pos, "-", "-", "-", "-"))
                continue
            az = [m["azimuth_deg"] for m in finals if m["doa_reliable"]]
            mean = circ_mean(az) if az else float("nan")
            err = abs((mean - EXPECTED_DEG.get(pos, mean) + 180) % 360 - 180)
            who, n = Counter(m["speaker"] for m in finals).most_common(1)[0]
            score = float(np.median([m["score"] for m in finals]))
            rows.append(
                (
                    pos,
                    f"{who} ({n}/{len(finals)})",
                    f"{mean:+.0f}",
                    f"{err:.0f}",
                    f"{score:.2f}",
                )
            )
        print(
            f"\n{BOLD}{'position':<10}{'speaker':<18}{'azimuth':>8}{'error':>7}{'score':>7}{RESET}"
        )
        for r in rows:
            print(f"{r[0]:<10}{r[1]:<18}{r[2]:>8}{r[3]:>7}{r[4]:>7}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("watch")
    for name in ("enroll", "lookback"):
        p = sub.add_parser(name)
        p.add_argument("name")
        p.add_argument("--seconds", type=float, default=20.0)
    sub.add_parser("target").add_argument("name", nargs="?", default="")
    sub.add_parser("forget").add_argument("name")
    sub.add_parser("list")
    s = sub.add_parser("session")
    s.add_argument("--positions", default="front,right,left")
    s.add_argument("--secs", type=float, default=15.0)
    s.add_argument("--grace", type=float, default=6.0, help="wait for late finals")
    args = ap.parse_args()

    rclpy.init()
    mon = Monitor()
    try:
        if args.cmd == "watch":
            mon.verbose = True
            print(f"{BOLD}who / where{RESET}  (0 deg = front, +90 left, -90 right)")
            mon.spin(float("inf"))
        elif args.cmd == "enroll":
            mon.command({"op": "enroll_start"}, "enroll_start")
            print(f"\n>>> Speak now for {args.seconds:.0f} s (normal sentences)\n")
            mon.spin(args.seconds)
            print(">>> Stop talking, saving...")
            mon.spin(1.5)  # let the last segment close
            mon.command({"op": "enroll_commit", "name": args.name}, "enroll_commit", 20)
        elif args.cmd == "lookback":
            payload = {
                "op": "enroll_lookback",
                "name": args.name,
                "seconds": args.seconds,
            }
            mon.command(payload, "enroll_commit", 20)
        elif args.cmd == "target":
            mon.command({"op": "set_target", "name": args.name}, "set_target")
        elif args.cmd == "forget":
            mon.command({"op": "forget", "name": args.name}, "forget")
        elif args.cmd == "list":
            mon.command({"op": "list"}, "list")
        else:
            mon.session(args.positions.split(","), args.secs, args.grace)
    except KeyboardInterrupt:
        pass
    finally:
        mon.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
