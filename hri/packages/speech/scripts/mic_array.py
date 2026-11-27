#!/usr/bin/env python3
"""ReSpeaker 4-mic array node: MVDR beam for the speaker id."""

import os

# Stop OpenBLAS threads spinning (~50 % of an Orin core); set before numpy
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import time  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from frida_interfaces.msg import AudioData  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from speech.mic_array_utils import FrontEndConfig, MvdrFrontEnd  # noqa: E402
from speech.silero_vad_utils import SileroVAD, default_silero_path  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

SAMPLE_RATE = 16000


class MicArray(Node):
    def __init__(self):
        super().__init__("mic_array")
        self.declare_parameter("MULTICHANNEL_AUDIO_TOPIC", "/hri/rawAudioMultichannel")
        self.declare_parameter(
            "ARRAY_PROCESSED_AUDIO_TOPIC", "/hri/arrayProcessedAudioChunk"
        )
        self.declare_parameter("SAYING_TOPIC", "/saying")
        self.declare_parameter("MC_CHANNELS", 6)
        self.declare_parameter("OUTPUT_GAIN_DB", 10.0)
        self.declare_parameter("TTS_TAIL_SEC", 0.3)
        self.declare_parameter("SILERO_MODEL", "")
        in_topic = self.get_parameter("MULTICHANNEL_AUDIO_TOPIC").value
        out_topic = self.get_parameter("ARRAY_PROCESSED_AUDIO_TOPIC").value
        self.n_ch = self.get_parameter("MC_CHANNELS").value
        self.tts_tail = self.get_parameter("TTS_TAIL_SEC").value
        silero = self.get_parameter("SILERO_MODEL").value or default_silero_path()
        self.front_end = MvdrFrontEnd(
            SileroVAD(silero, SAMPLE_RATE),
            FrontEndConfig(output_gain_db=self.get_parameter("OUTPUT_GAIN_DB").value),
        )
        self._saying = False
        self._muted_until = 0.0
        self.audio_pub = self.create_publisher(AudioData, out_topic, 20)
        self.create_subscription(AudioData, in_topic, self._audio_cb, 50)
        self.create_subscription(
            Bool, self.get_parameter("SAYING_TOPIC").value, self._saying_cb, 10
        )
        self.get_logger().info("MicArray ready")
        self.get_logger().debug(f"MicArray | in: {in_topic} | out: {out_topic}")

    def _saying_cb(self, msg: Bool) -> None:
        self._saying = bool(msg.data)
        if not msg.data:
            self._muted_until = time.monotonic() + self.tts_tail

    def _audio_cb(self, msg: AudioData) -> None:
        x = np.frombuffer(bytes(msg.data), dtype=np.int16)
        x = x[: x.shape[0] - x.shape[0] % self.n_ch].reshape(-1, self.n_ch)
        saying = self._saying or time.monotonic() < self._muted_until
        y = self.front_end.process(x, saying)
        if y.size:
            self.audio_pub.publish(
                AudioData(data=(y * 32767).astype(np.int16).tobytes())
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
