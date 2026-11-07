"""Streaming Silero VAD over onnxruntime (supports the v4 h/c and v5 state graphs)."""

from __future__ import annotations

import numpy as np

FRAME_SAMPLES = 512  # 32 ms at 16 kHz; fixed by the model
_V5_CONTEXT = 64


class SileroVAD:
    """Stateful Silero VAD. Call with consecutive 512-sample float32 frames."""

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
