#!/usr/bin/env python3
"""Guided live check of "who is speaking and from where" (talker on the robot).

Prompts the talker to speak from a few positions; for each one it records the
/hri/speaker_id/located messages and summarises speaker + direction, then
prints a table. Needs audio_capturer + speaker_verification running.

    python3 live_session.py [--positions front,right,left] [--secs 12] [--wait 8] [--manual]
"""

import argparse
import json
import time
from collections import Counter

import numpy as np
import rclpy
from speech.spatial.doa import circ_diff
from std_msgs.msg import String

EXPECTED = {
    "front": 0,
    "left": 90,
    "right": -90,
    "back": 180,
    "front-left": 45,
    "front-right": -45,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--positions", default="front,right,left")
    ap.add_argument("--secs", type=float, default=12)
    ap.add_argument("--wait", type=float, default=8)
    ap.add_argument("--grace", type=float, default=6, help="wait for late finals")
    ap.add_argument(
        "--manual", action="store_true", help="wait for ENTER before each position"
    )
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("live_session")
    msgs = []
    node.create_subscription(
        String, "/hri/speaker_id/located", lambda m: msgs.append(json.loads(m.data)), 50
    )
    rows = []
    for pos in args.positions.split(","):
        print(f"\n\033[1m>>> Go to the robot's {pos.upper()} (~0.8 m).\033[0m")
        if args.manual:
            input("    press ENTER when you are in place... ")
        t_end = time.time() + args.wait
        while time.time() < t_end:
            print(f"    starting in {t_end - time.time():4.0f} s", end="\r", flush=True)
            rclpy.spin_once(node, timeout_sec=0.2)
        msgs.clear()
        print(
            f"\n\033[1m>>> SPEAK NOW ({args.secs:.0f} s) from the {pos.upper()}\033[0m"
        )
        t_end = time.time() + args.secs
        while time.time() < t_end:
            print(f"    {t_end - time.time():4.0f} s left", end="\r", flush=True)
            rclpy.spin_once(node, timeout_sec=0.2)
        # segments still open when the window closes publish their final
        # (per-voice) result a bit later: wait for those too
        seen = {m["seg_id"] for m in msgs}
        t_end = time.time() + args.grace
        while time.time() < t_end and not seen <= {
            m["seg_id"] for m in msgs if m["final"]
        }:
            rclpy.spin_once(node, timeout_sec=0.2)
        finals = [m for m in msgs if m["final"] and m["seg_id"] in seen] or msgs
        if not finals:
            rows.append((pos, 0, "-", "-", "-", "-"))
            print("\n    (no speech detected)")
            continue
        # a MIXED segment is split per voice: judge the position by the named voice
        named = [m for m in finals if m["speaker"] not in ("", "none", None)]
        if named:
            finals = named
        az = np.array([m["azimuth_deg"] for m in finals if m["doa_reliable"]])
        med = (
            float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(az))))))
            if az.size
            else float("nan")
        )
        who = Counter(m["speaker"] for m in finals).most_common(1)[0]
        dirs = Counter(m["direction"] for m in finals).most_common(1)[0][0]
        err = (
            abs(float(circ_diff(med, EXPECTED.get(pos, med))))
            if az.size
            else float("nan")
        )
        scores = [m["score"] for m in finals]
        rows.append(
            (
                pos,
                len(finals),
                f"{who[0]} ({who[1]}/{len(finals)})",
                dirs,
                f"{med:+.0f} (err {err:.0f})",
                f"{np.median(scores):.2f}",
            )
        )
        print(f"\n    -> {rows[-1]}")
    print("\n\033[1mposition  n  speaker  direction  azimuth  median score\033[0m")
    for r in rows:
        print("  ".join(str(c) for c in r))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
