#!/usr/bin/env python3
"""Spatial audio front end node: ReSpeaker raw mics -> one clean mono stream.

    MULTICHANNEL_AUDIO_TOPIC (6 ch int16) -> speech.spatial.frontend
        (100 Hz HPF, Silero VAD, RTF-MVDR, streaming DeepFilterNet3)
    -> ENHANCED_AUDIO_TOPIC (mono int16, 1024-sample chunks)

Replaces noise_cancellation (per-chunk DFN on the chip's ch0) when the HRI
stack is launched with HRI_AUDIO_FRONTEND=spatial; then ENHANCED_AUDIO_TOPIC is
the processed topic that voice_detection / KWS / STT already read.
"""

import os
import queue
import threading
import time

import numpy as np
import rclpy
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.spatial.beamformer import MvdrConfig
from speech.spatial.denoise import StreamingConfig, make_dfn_enhance
from speech.spatial.frontend import FrontEndConfig, SpatialFrontEnd
from speech.speaker_id.silero import SileroVAD
from std_msgs.msg import Bool

SR = 16000
OUT_CHUNK = 1024
SLOW_LIMIT = 30  # consecutive slower-than-real-time blocks before DFN is dropped


def _speech_dir() -> str:
    src = "/workspace/src/hri/packages/speech"
    if os.path.isdir(src):
        return src
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory("speech")


class SpatialAudio(Node):
    def __init__(self):
        super().__init__("spatial_audio")
        defaults = {
            "MULTICHANNEL_AUDIO_TOPIC": "/hri/rawAudioMultichannel",
            "ENHANCED_AUDIO_TOPIC": "/hri/enhancedAudioChunk",
            "SAYING_TOPIC": "/saying",
            "MC_CHANNELS": 6,
            "USE_MVDR": True,
            "USE_DFN": True,
            "DF_ATTEN_LIM_DB": 6.0,
            "DF_MODEL_PATH": "assets/downloads",
            "DFN_BLOCK_SEC": 0.256,
            "DFN_CONTEXT_SEC": 1.0,
            "HPF_HZ": 100.0,
            "VAD_INPUT_GAIN_DB": 12.0,
            "OUTPUT_GAIN_DB": 12.0,
            "MVDR_NOISE_TAU_SEC": 3.0,
            "MVDR_SPEECH_TAU_SEC": 1.5,
            "STATS_PERIOD_SEC": 30.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in defaults}
        self.p = p

        cfg = FrontEndConfig(
            hpf_hz=p["HPF_HZ"],
            use_mvdr=p["USE_MVDR"],
            output_gain_db=p["OUTPUT_GAIN_DB"],
            vad_input_gain_db=p["VAD_INPUT_GAIN_DB"],
            mvdr=MvdrConfig(
                noise_tau_sec=p["MVDR_NOISE_TAU_SEC"],
                speech_tau_sec=p["MVDR_SPEECH_TAU_SEC"],
            ),
            dfn=StreamingConfig(
                block_sec=p["DFN_BLOCK_SEC"], context_sec=p["DFN_CONTEXT_SEC"]
            ),
        )
        base = _speech_dir()
        self.fe = SpatialFrontEnd(
            SileroVAD(os.path.join(base, "assets", "silero_vad.onnx")), cfg
        )
        self.n_ch = p["MC_CHANNELS"]
        self._saying = False
        self._q: queue.Queue = queue.Queue(maxsize=50)
        self._out = np.zeros(0, dtype=np.int16)
        self._drops = 0
        self._slow = 0
        self._proc_sec = 0.0
        self._audio_sec = 0.0

        self.pub = self.create_publisher(AudioData, p["ENHANCED_AUDIO_TOPIC"], 20)
        self.create_subscription(AudioData, p["MULTICHANNEL_AUDIO_TOPIC"], self._cb, 50)
        self.create_subscription(Bool, p["SAYING_TOPIC"], self._saying_cb, 10)
        self.create_timer(p["STATS_PERIOD_SEC"], self._stats)
        threading.Thread(target=self._loop, daemon=True).start()
        if p["USE_DFN"]:
            threading.Thread(
                target=self._load_dfn,
                args=(os.path.join(base, p["DF_MODEL_PATH"]),),
                daemon=True,
            ).start()
        self.get_logger().info(
            f"SpatialAudio ready | {p['MULTICHANNEL_AUDIO_TOPIC']} -> "
            f"{p['ENHANCED_AUDIO_TOPIC']} | MVDR {p['USE_MVDR']} | DFN "
            f"{'loading' if p['USE_DFN'] else 'off'} (atten {p['DF_ATTEN_LIM_DB']} dB)"
        )

    def _load_dfn(self, model_base):
        try:
            import df as df_module
            import torch

            model_dir = os.path.join(model_base, "DeepFilterNet3")
            model, state, _ = df_module.init_df(
                model_base_dir=model_dir, log_level="none", log_file=None
            )
            dev = "cpu"
            if torch.cuda.is_available():
                model = model.to("cuda")
                dev = "cuda"
            fn = make_dfn_enhance(model, state, df_module, self.p["DF_ATTEN_LIM_DB"])
            for _ in range(3):  # warm-up at the real block size
                fn(
                    np.zeros(
                        3
                        * int(
                            (self.p["DFN_BLOCK_SEC"] + self.p["DFN_CONTEXT_SEC"]) * SR
                        ),
                        np.float32,
                    )
                )
            self.fe.set_enhance_fn(fn)
            self.get_logger().info(f"DeepFilterNet3 streaming active ({dev})")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"DeepFilterNet unavailable, MVDR only: {exc!r}")

    def _saying_cb(self, msg: Bool):
        self._saying = bool(msg.data)

    def _cb(self, msg: AudioData):
        try:
            self._q.put_nowait((bytes(msg.data), self._saying))
        except queue.Full:
            self._drops += 1

    def _loop(self):
        while rclpy.ok():
            try:
                data, saying = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            x = np.frombuffer(data, dtype=np.int16)
            x = x[: x.shape[0] - x.shape[0] % self.n_ch].reshape(-1, self.n_ch)
            t0 = time.monotonic()
            y = self.fe.process(x, saying)
            dt = time.monotonic() - t0
            dur = x.shape[0] / SR
            self._proc_sec += dt
            self._audio_sec += dur
            if self.fe.den is not None:
                self._slow = self._slow + 1 if (dt > dur and self._q.qsize() > 5) else 0
                if self._slow >= SLOW_LIMIT:
                    self.get_logger().warn(
                        "DeepFilterNet cannot keep up with real time; MVDR only from now on"
                    )
                    self.fe.set_enhance_fn(None)
            self._out = np.concatenate([self._out, (y * 32767).astype(np.int16)])
            while self._out.shape[0] >= OUT_CHUNK:
                chunk = self._out[:OUT_CHUNK]
                self._out = self._out[OUT_CHUNK:]
                self.pub.publish(AudioData(data=chunk.tobytes()))

    def _stats(self):
        rtf = self._proc_sec / self._audio_sec if self._audio_sec else 0.0
        self.get_logger().info(
            f"rtf={rtf:.2f} mvdr_ready={self.fe.bf.ready if self.fe.bf else 'off'} "
            f"dfn={'on' if self.fe.den else 'off'} drops={self._drops} "
            f"latency={self.fe.latency_sec * 1000:.0f}ms"
        )
        self._proc_sec = self._audio_sec = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SpatialAudio()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
