"""Streaming Silero VAD."""

from __future__ import annotations

import os

import numpy as np

FRAME_SAMPLES = 512
_V5_CONTEXT = 64
SRC_SPEECH_DIR = "/workspace/src/hri/packages/speech"


def default_silero_path() -> str:
    candidates = [os.path.join(SRC_SPEECH_DIR, "assets", "silero_vad.onnx")]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            os.path.join(
                get_package_share_directory("speech"), "assets", "silero_vad.onnx"
            )
        )
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


class SileroVAD:
    def __init__(self, model_path: str, sample_rate: int = 16000) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        opts.log_severity_level = 3
        self.session = ort.InferenceSession(
            model_path, sess_options=opts, providers=["CPUExecutionProvider"]
        )
        names = {i.name for i in self.session.get_inputs()}
        self.is_v5 = "state" in names
        self.sample_rate = sample_rate
        self._sr = np.array(sample_rate, dtype=np.int64)
        self.reset()

    def reset(self) -> None:
        if self.is_v5:
            self._state = np.zeros((2, 1, 128), dtype=np.float32)
            self._context = np.zeros(_V5_CONTEXT, dtype=np.float32)
        else:
            self._h = np.zeros((2, 1, 64), dtype=np.float32)
            self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def __call__(self, frame: np.ndarray) -> float:
        frame = np.asarray(frame, dtype=np.float32)
        if frame.shape[0] != FRAME_SAMPLES:
            raise ValueError(f"Silero expects {FRAME_SAMPLES} samples per frame")
        if self.is_v5:
            x = np.concatenate([self._context, frame])[None, :]
            out, self._state = self.session.run(
                None, {"input": x, "state": self._state, "sr": self._sr}
            )
            self._context = x[0, -_V5_CONTEXT:]
        else:
            out, self._h, self._c = self.session.run(
                None,
                {"input": frame[None, :], "h": self._h, "c": self._c, "sr": self._sr},
            )
        return float(np.asarray(out).reshape(-1)[0])


class VadGate:
    def __init__(
        self,
        vad,
        sample_rate: int = 16000,
        on: float = 0.5,
        off: float = 0.35,
        hangover_sec: float = 0.3,
    ) -> None:
        self.vad = vad
        self.on, self.off = on, off
        self._hang = int(hangover_sec * sample_rate / FRAME_SAMPLES)
        self._buf = np.zeros(0, dtype=np.float32)
        self._off_frames = 0
        self.speech = False

    def update(self, mono: np.ndarray, saying: bool = False) -> bool:
        self._buf = np.concatenate([self._buf, mono])
        while self._buf.shape[0] >= FRAME_SAMPLES:
            p = self.vad(self._buf[:FRAME_SAMPLES])
            self._buf = self._buf[FRAME_SAMPLES:]
            if p >= self.on:
                self.speech = True
                self._off_frames = 0
            elif p < self.off and self.speech:
                self._off_frames += 1
                if self._off_frames > self._hang:
                    self.speech = False
        if saying:
            self.speech = False
        return self.speech
