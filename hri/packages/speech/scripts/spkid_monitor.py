#!/usr/bin/env python3
"""Live terminal monitor for speaker verification.

    ros2 run speech spkid_monitor.py [--csv /path/results.csv] [--scenario S1] [--partials]

Shows one line per decision (final segments; partials with --partials) and a
status line with the Silero probability, both VADs (new and legacy
/hri/voice_activity), target_active, TTS mute, noise floor and latency.
With --csv, every final decision is appended with the scenario tag so runs can
be compared later.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import time

import rclpy
from frida_constants.hri_constants import (
    SAYING_TOPIC,
    SPEAKER_ID_METRICS_TOPIC,
    SPEAKER_ID_RESULT_TOPIC,
    SPEAKER_ID_STATUS_TOPIC,
    SPEAKER_ID_TARGET_ACTIVE_TOPIC,
    SPEAKER_ID_VAD_TOPIC,
    VOICE_ACTIVITY_TOPIC,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

COLORS = {
    "TARGET": "\033[92m",
    "NOT_TARGET": "\033[91m",
    "MIXED": "\033[95m",
    "UNKNOWN": "\033[93m",
    "NO_PROFILES": "\033[90m",
    "TTS_CUT": "\033[90m",
    "TOO_SHORT": "\033[90m",
}
RESET = "\033[0m"
CSV_FIELDS = [
    "wall_time",
    "scenario",
    "seg_id",
    "dur",
    "label",
    "target",
    "target_score",
    "best",
    "score",
    "second",
    "second_score",
    "vad_p",
    "snr_db",
    "infer_ms",
    "lat_ms",
    "whole_label",
    "win_n",
    "win_target",
    "win_not",
    "win_max",
    "legacy_vad_seen",
    "enrolling",
    "tts_cut",
    "src",
]


class Monitor(Node):
    def __init__(self, args) -> None:
        super().__init__("spkid_monitor")
        self.args = args
        self.state = {
            "vad": False,
            "legacy": False,
            "target": False,
            "saying": False,
            "legacy_seen": False,
        }
        self.metrics: dict = {}
        self.csv_writer = None
        if args.csv:
            new = not os.path.exists(args.csv)
            self.csv_file = open(args.csv, "a", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_file, fieldnames=CSV_FIELDS, extrasaction="ignore"
            )
            if new:
                self.csv_writer.writeheader()
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, SPEAKER_ID_RESULT_TOPIC, self._result, 50)
        self.create_subscription(String, SPEAKER_ID_METRICS_TOPIC, self._metrics, 10)
        self.create_subscription(String, SPEAKER_ID_STATUS_TOPIC, self._status, latched)
        self.create_subscription(Bool, SPEAKER_ID_VAD_TOPIC, self._flag("vad"), 10)
        self.create_subscription(Bool, VOICE_ACTIVITY_TOPIC, self._legacy, 10)
        self.create_subscription(
            Bool, SPEAKER_ID_TARGET_ACTIVE_TOPIC, self._flag("target"), 10
        )
        self.create_subscription(Bool, SAYING_TOPIC, self._flag("saying"), 10)
        self.create_timer(0.2, self._status_line)
        print(
            f"Monitoring {SPEAKER_ID_RESULT_TOPIC} "
            f"(scenario={args.scenario or '-'}, csv={args.csv or 'off'})\n"
        )

    def _flag(self, key: str):
        def cb(msg: Bool) -> None:
            self.state[key] = bool(msg.data)

        return cb

    def _legacy(self, msg: Bool) -> None:
        self.state["legacy"] = bool(msg.data)
        if msg.data:
            self.state["legacy_seen"] = True

    def _metrics(self, msg: String) -> None:
        self.metrics = json.loads(msg.data)

    def _status(self, msg: String) -> None:
        d = json.loads(msg.data)
        if d["op"] in ("enroll_progress",):
            text = d["message"]
        else:
            text = f"{d['op']}: {d['message']}"
        profs = ", ".join(p["name"] for p in d.get("profiles", [])) or "none"
        self._print(
            f"\033[96m[status] {text} | target={d['target'] or 'any'} | profiles={profs}{RESET}"
        )

    def _result(self, msg: String) -> None:
        r = json.loads(msg.data)
        if not r["final"] and not self.args.partials:
            return
        label = r.get("label", "?")
        color = COLORS.get(label, "")
        kind = "FINAL  " if r["final"] else "partial"
        if "score" in r:
            body = (
                f"best={r['best'] or '-'}:{r['score']:+.2f}  "
                f"2nd={r['second'] or '-'}:{r['second_score']:+.2f}  "
                f"tgt={r['target'] or 'any'}:{r['target_score']:+.2f}  "
                f"infer={r.get('infer_ms', 0):.0f}ms"
            )
            if r["final"] and "win_n" in r:
                body += f"  win T/N/all={r['win_target']}/{r['win_not']}/{r['win_n']}"
        else:
            body = ""
        lat = (
            f"lat={r['lat_ms']:.0f}ms"
            if "lat_ms" in r
            else f"+{r.get('since_onset_ms', 0):.0f}ms"
        )
        self._print(
            f"{time.strftime('%H:%M:%S')} {kind} seg{r['seg_id']:<4} {r['dur']:4.1f}s "
            f"{color}{label:<10}{RESET} {body}  snr={r['snr_db']:.0f}dB {lat}"
            + ("  [enrolling]" if r.get("enrolling") else "")
        )
        if r["final"] and self.csv_writer:
            row = dict(r)
            row["wall_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
            row["scenario"] = self.args.scenario
            row["legacy_vad_seen"] = self.state["legacy_seen"]
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        if r["final"]:
            self.state["legacy_seen"] = self.state["legacy"]

    def _print(self, line: str) -> None:
        print("\r\033[K" + line, flush=True)

    def _status_line(self) -> None:
        m = self.metrics
        prob = float(m.get("vad_prob", 0.0))
        bar = "#" * int(prob * 20)

        def onoff(key: str, name: str) -> str:
            return (
                f"\033[92m{name}\033[0m"
                if self.state[key]
                else f"\033[90m{name}\033[0m"
            )

        print(
            "\r\033[K"
            f"p={prob:.2f} [{bar:<20}] {onoff('vad', 'VAD')} {onoff('legacy', 'oldVAD')} "
            f"{onoff('target', 'TARGET')} {onoff('saying', 'TTS')} "
            f"floor={m.get('noise_floor_db', 0):.0f}dB "
            f"infer p95={m.get('infer_ms_p95', 0):.0f}ms "
            f"tgt={m.get('target') or 'any'} enroll={'ON' if m.get('enrolling') else 'off'}"
            + (
                f" {m.get('enroll_collected_sec', 0):.1f}s"
                if m.get("enrolling")
                else ""
            ),
            end="",
            flush=True,
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="")
    ap.add_argument("--scenario", default="")
    ap.add_argument("--partials", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = Monitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
