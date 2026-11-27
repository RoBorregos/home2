#!/usr/bin/env python3

"""Simulated guest: answers FRIDA through the real STT pipeline, so a task can run with nobody
in front of the robot. It reads what FRIDA says (/speech/text_spoken), waits for the robot to
listen (AudioState == "listening") and streams a recorded answer into the VAD audio topic that
hear_streaming transcribes. It prints the conversation as it happens.

    task_manager/scripts/test/sim_hric.sh run 1
    (or: python3 sim_guest.py --ros-args -p answer_on_try:=1 -p wav_dir:=<dir>)

answer_on_try: 1 = answers at once, 3 = only on the 3rd time a question is asked (forces
retries), 0 = never answers (silent guest). WAVs (16 kHz mono int16, not in git) default to
sim_guest_wavs/ next to this file.
"""

import os
import threading
import time
import wave

import numpy as np
import rclpy
from frida_constants.hri_constants import VAD_AUDIO_TOPIC, VOICE_ACTIVITY_TOPIC
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

CHUNK = 1024  # samples per message, like audio_capturer
RATE = 16000
VAD_SILENCE_LIMIT = 1.5  # voice_detection forwards this much silence before voice_activity=False


def pick_answer(said: str, guest: int) -> str | None:
    """Which recording answers what FRIDA just said; None = stay quiet."""
    s = said.lower()
    if "did you say" in s:
        return "yes"
    if "name" in s and "?" in s:
        return f"g{guest}_name"
    if "drink" in s and "?" in s:
        return f"g{guest}_drink"
    if "where would you like me to go" in s:
        return "dest"
    if "say stop" in s or "following you" in s:
        return "stop"
    return None


class SimGuest(Node):
    def __init__(self):
        super().__init__("sim_guest")
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_guest_wavs")
        wav_dir = self.declare_parameter("wav_dir", default_dir).value
        self.answer_on_try = self.declare_parameter("answer_on_try", 1).value
        self.reply_delay = self.declare_parameter("reply_delay", 1.0).value
        self.wavs = {}
        for f in os.listdir(wav_dir):
            if f.endswith(".wav"):
                with wave.open(os.path.join(wav_dir, f)) as w:
                    assert w.getframerate() == RATE and w.getnchannels() == 1, f
                    self.wavs[f[:-4]] = np.frombuffer(w.readframes(w.getnframes()), np.int16)

        self.guest = 1
        self.last_said = ""
        self.tries = {}
        self.last_heard = ""
        self.t0 = time.monotonic()
        self.audio_pub = self.create_publisher(AudioData, VAD_AUDIO_TOPIC, 20)
        self.activity_pub = self.create_publisher(Bool, VOICE_ACTIVITY_TOPIC, 10)
        self.create_subscription(String, "/speech/text_spoken", self._said, 10)
        self.create_subscription(String, "AudioState", self._state, 10)
        self.create_subscription(String, "/speech/raw_command", self._heard, 10)
        self.log(f"ready, answer_on_try={self.answer_on_try}, clips={sorted(self.wavs)}")

    def log(self, text: str):
        print(f"[{time.monotonic() - self.t0:6.1f}s] {text}", flush=True)

    def _said(self, msg: String):
        self.last_said = msg.data
        self.log(f"FRIDA : {msg.data}")
        if "nice to meet you" in msg.data.lower():
            self.guest = min(self.guest + 1, 2)  # the next name/drink answers come from guest 2

    def _heard(self, msg: String):
        if msg.data != self.last_heard:
            self.last_heard = msg.data
            self.log(f"  stt : {msg.data}")

    def _state(self, msg: String):
        if msg.data != "listening":
            return
        clip = pick_answer(self.last_said, self.guest)
        key = (self.guest, self.last_said)
        self.tries[key] = self.tries.get(key, 0) + 1
        if clip is None or clip not in self.wavs:
            return
        if self.answer_on_try == 0 or self.tries[key] < self.answer_on_try:
            self.log(f"GUEST : (silent, try {self.tries[key]})")
            return
        threading.Thread(target=self._speak, args=(clip,), daemon=True).start()

    def _speak(self, clip: str):
        time.sleep(self.reply_delay)  # LISTEN is published before the STT goal starts
        self.log(f"GUEST : <{clip}>")
        audio = np.concatenate([self.wavs[clip], np.zeros(int(RATE * VAD_SILENCE_LIMIT), np.int16)])
        for i in range(0, len(audio), CHUNK):
            self.audio_pub.publish(AudioData(data=audio[i : i + CHUNK].tobytes()))
            time.sleep(CHUNK / RATE)
        self.activity_pub.publish(Bool(data=False))  # end of utterance, like the VAD


def main():
    rclpy.init()
    node = SimGuest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    assert pick_answer("Did you say Charlie? Yes or no?", 1) == "yes"
    assert pick_answer("What is your name?", 2) == "g2_name"
    assert pick_answer("What is your favorite drink?", 1) == "g1_drink"
    assert pick_answer("Nice to meet you, Charlie.", 1) is None
    main()
