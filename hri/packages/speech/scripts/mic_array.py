#!/usr/bin/env python3
"""ReSpeaker 4-mic array node: MVDR beam, sound direction and who + where."""

import os

# Stop OpenBLAS threads spinning (~50 % of an Orin core); set before numpy
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import json  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from frida_interfaces.msg import AudioData  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from speech.mic_array_utils import (  # noqa: E402
    ArrayGeometry,
    DoaConfig,
    FrontEndConfig,
    MvdrFrontEnd,
    SrpPhat,
    direction_label,
)
from speech.silero_vad_utils import SileroVAD, default_silero_path  # noqa: E402
from speech.speaker_localization_utils import (  # noqa: E402
    DirectionBuffer,
    SpeakerTracker,
    TrackerConfig,
)
from std_msgs.msg import Bool, String  # noqa: E402

SAMPLE_RATE = 16000
NO_DIRECTION = {
    "azimuth_deg": None,
    "direction": None,
    "confidence": 0.0,
    "reliable": False,
    "sources": [],
}


class MicArray(Node):
    def __init__(self):
        super().__init__("mic_array")
        self.declare_parameter("MULTICHANNEL_AUDIO_TOPIC", "/hri/rawAudioMultichannel")
        self.declare_parameter(
            "ARRAY_PROCESSED_AUDIO_TOPIC", "/hri/arrayProcessedAudioChunk"
        )
        self.declare_parameter("SOUND_DIRECTION_TOPIC", "/hri/speech/sound_direction")
        self.declare_parameter("SPEAKER_ID_TOPIC", "/hri/speaker_id/decision")
        self.declare_parameter("SPEAKER_ID_LOCATED_TOPIC", "/hri/speaker_id/located")
        self.declare_parameter("SAYING_TOPIC", "/saying")
        self.declare_parameter("MC_CHANNELS", 6)
        self.declare_parameter("OUTPUT_GAIN_DB", 10.0)
        self.declare_parameter("TTS_TAIL_SEC", 0.3)
        self.declare_parameter("SILERO_MODEL", "")
        self.declare_parameter("DOA_OFFSET_DEG", 0.0)
        self.declare_parameter("DOA_MIRROR", False)
        self.declare_parameter("DOA_MIN_CONF", 0.15)
        self.declare_parameter("DOA_WINDOW_SEC", 1.0)
        self.declare_parameter("DOA_HOP_SEC", 0.25)
        self.declare_parameter("TRACK_MAX_AGE_SEC", 8.0)
        self.declare_parameter("TRACK_MATCH_DEG", 25.0)

        def param(name):
            return self.get_parameter(name).value

        in_topic = param("MULTICHANNEL_AUDIO_TOPIC")
        self.n_ch = param("MC_CHANNELS")
        self.tts_tail = param("TTS_TAIL_SEC")
        silero = param("SILERO_MODEL") or default_silero_path()
        self.front_end = MvdrFrontEnd(
            SileroVAD(silero, SAMPLE_RATE),
            FrontEndConfig(output_gain_db=param("OUTPUT_GAIN_DB")),
        )
        self.min_conf = param("DOA_MIN_CONF")
        self.doa_window_sec = param("DOA_WINDOW_SEC")
        self.doa_hop = int(param("DOA_HOP_SEC") * SAMPLE_RATE)
        geometry = ArrayGeometry(
            offset_deg=float(param("DOA_OFFSET_DEG")), mirror=bool(param("DOA_MIRROR"))
        )
        self.srp = SrpPhat(
            DoaConfig(sample_rate=SAMPLE_RATE, geometry=geometry),
            history_sec=self.doa_window_sec + 1.0,
        )
        self._since_doa = 0
        self.directions = DirectionBuffer()
        self.tracker = SpeakerTracker(
            TrackerConfig(
                max_age_sec=param("TRACK_MAX_AGE_SEC"),
                match_deg=param("TRACK_MATCH_DEG"),
            )
        )
        self._saying = False
        self._muted_until = 0.0
        self.audio_pub = self.create_publisher(
            AudioData, param("ARRAY_PROCESSED_AUDIO_TOPIC"), 20
        )
        self.direction_pub = self.create_publisher(
            String, param("SOUND_DIRECTION_TOPIC"), 20
        )
        self.located_pub = self.create_publisher(
            String, param("SPEAKER_ID_LOCATED_TOPIC"), 10
        )
        self.create_subscription(AudioData, in_topic, self._audio_cb, 50)
        self.create_subscription(Bool, param("SAYING_TOPIC"), self._saying_cb, 10)
        self.create_subscription(
            String, param("SPEAKER_ID_TOPIC"), self._speaker_id_cb, 20
        )
        self.get_logger().info("MicArray ready")
        self.get_logger().debug(
            f"MicArray | in: {in_topic} | DOA offset={geometry.offset_deg} "
            f"mirror={geometry.mirror}"
        )

    def _saying_cb(self, msg: Bool) -> None:
        self._saying = bool(msg.data)
        if not msg.data:
            self._muted_until = time.monotonic() + self.tts_tail

    def _audio_cb(self, msg: AudioData) -> None:
        x = np.frombuffer(bytes(msg.data), dtype=np.int16)
        x = x[: x.shape[0] - x.shape[0] % self.n_ch].reshape(-1, self.n_ch)
        saying = self._saying or time.monotonic() < self._muted_until
        self.srp.set_speech(self.front_end.speech)
        self.srp.push(x[:, 1:5].astype(np.float32) / 32768.0)
        y = self.front_end.process(x, saying)
        if y.size:
            self.audio_pub.publish(
                AudioData(data=(y * 32767).astype(np.int16).tobytes())
            )
        self._since_doa += x.shape[0]
        if self._since_doa >= self.doa_hop:
            self._since_doa = 0
            self._publish_direction(time.time())

    def _publish_direction(self, t: float) -> None:
        est = self.srp.localize(int(self.doa_window_sec * SAMPLE_RATE))
        if est is None:
            return
        out = est.as_dict()
        out.update(
            {
                "t": round(t, 3),
                "window_sec": self.doa_window_sec,
                "speech": self.front_end.speech,
                "direction": direction_label(est.azimuth_deg),
                "reliable": est.confidence >= self.min_conf,
            }
        )
        self.directions.add(out)
        self.direction_pub.publish(String(data=json.dumps(out)))

    def _speaker_id_cb(self, msg: String) -> None:
        d = json.loads(msg.data)
        if "score" not in d:
            return
        if d["final"] and d.get("parts") and self._publish_parts(d):
            return
        doa = self.directions.direction([(d["t_start"], d["t_end"])], self.min_conf)
        self._publish_located(d, doa)

    def _publish_parts(self, d: dict) -> bool:
        published = False
        for part in d["parts"]:
            doa = self.directions.direction(part["windows"], self.min_conf)
            if doa is None:
                continue
            self._publish_located({**d, **part, "second_score": -1.0}, doa)
            self.get_logger().info(
                f"[seg {d['seg_id']}]   {part['part']:6s} voice: "
                f"{part['best'] or '?'}:{part['score']:.2f} "
                f"x{len(part['windows'])} windows | DOA {doa['azimuth_deg']:+.0f}deg "
                f"{doa['direction']} conf={doa['confidence']:.2f}"
            )
            published = True
        return published

    def _publish_located(self, d: dict, doa: dict | None) -> None:
        doa = doa or NO_DIRECTION
        now = time.time()
        known = bool(d.get("known"))
        speaker, via = self.tracker.update(
            now,
            d.get("best") if known else None,
            known,
            doa["azimuth_deg"],
            doa["reliable"],
            rejected=bool(d.get("rejected")),
        )
        located = {
            "t": d["t"],
            "seg_id": d["seg_id"],
            "final": d["final"],
            "speaker": speaker,
            "via": via,
            "best": d.get("best", ""),
            "score": d.get("score", 0.0),
            "label": d.get("label", ""),
            "target": d.get("target", ""),
            "azimuth_deg": doa["azimuth_deg"],
            "direction": doa["direction"],
            "doa_confidence": doa["confidence"],
            "doa_reliable": doa["reliable"],
            "sources": doa["sources"],
            "second": d.get("second", ""),
            "second_score": d.get("second_score", 0.0),
            "dur": d["dur"],
            "snr_db": d["snr_db"],
            "part": d.get("part", "whole"),
            "tracks": self.tracker.snapshot(now),
        }
        self.located_pub.publish(String(data=json.dumps(located)))
        if d["final"]:
            where = (
                f"{doa['azimuth_deg']:+.0f}deg {doa['direction']}"
                if doa["azimuth_deg"] is not None
                else "no DOA"
            )
            self.get_logger().info(
                f"[seg {d['seg_id']}] {speaker} (via {via}) {where} "
                f"conf={doa['confidence']:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MicArray()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
