"""ONNX speaker-embedding model wrapper."""

from __future__ import annotations

import time

import numpy as np

from speech.speaker_id.features import speaker_features

DEFAULT_MODEL_FILE = "voxceleb_resnet34_LM.onnx"


def l2_normalize(vec: np.ndarray) -> np.ndarray:
    vec = np.asarray(vec, dtype=np.float32)
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 0 else vec


def pick_providers(requested: list[str] | None = None) -> list[str]:
    """Keep only the requested onnxruntime providers that are installed.

    ``requested`` uses short names (``"cuda"``, ``"tensorrt"``, ``"cpu"``) or full
    provider names. CPU is always appended as the last fallback.
    """
    import onnxruntime as ort

    aliases = {
        "cpu": "CPUExecutionProvider",
        "cuda": "CUDAExecutionProvider",
        "tensorrt": "TensorrtExecutionProvider",
        "trt": "TensorrtExecutionProvider",
    }
    available = set(ort.get_available_providers())
    wanted = [aliases.get(p.strip().lower(), p.strip()) for p in (requested or [])]
    providers = [p for p in wanted if p in available]
    if "CPUExecutionProvider" not in providers:
        providers.append("CPUExecutionProvider")
    return providers


class SpeakerEmbedder:
    """Computes L2-normalised speaker embeddings from 16 kHz float audio.

    Works with WeSpeaker ONNX exports (input ``feats [B, T, 80]``, output
    ``embs [B, D]``).
    """

    def __init__(
        self,
        model_path: str,
        providers: list[str] | None = None,
        sample_rate: int = 16000,
        num_threads: int = 2,
    ) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = num_threads
        opts.inter_op_num_threads = 1
        opts.log_severity_level = 3
        # Every segment has a different length; exhaustive cuDNN algo search would
        # re-run for each new shape (~60 ms instead of ~10 ms on the Orin).
        provider_cfg = [
            (p, {"cudnn_conv_algo_search": "HEURISTIC"})
            if p == "CUDAExecutionProvider"
            else p
            for p in pick_providers(providers)
        ]
        self.session = ort.InferenceSession(
            model_path, sess_options=opts, providers=provider_cfg
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.dim = int(self.session.get_outputs()[0].shape[-1])
        self.sample_rate = sample_rate
        self.providers = self.session.get_providers()
        self.last_infer_ms = 0.0

    def embed(self, wave_f32: np.ndarray) -> np.ndarray:
        """Embedding of a mono float waveform in [-1, 1] (at least ~0.5 s)."""
        start = time.perf_counter()
        feats = speaker_features(wave_f32, self.sample_rate)
        if feats.shape[0] < 10:
            raise ValueError("audio too short for a speaker embedding")
        out = self.session.run(
            [self.output_name], {self.input_name: feats[None, :, :]}
        )[0]
        self.last_infer_ms = (time.perf_counter() - start) * 1000.0
        return l2_normalize(out[0])

    def warmup(self) -> None:
        rng = np.random.default_rng(0)
        self.embed(0.01 * rng.standard_normal(self.sample_rate * 2).astype("float32"))
