"""Loads a frozen embedding backbone and embeds crops — shared by
gallery_build.py, the offline benchmark, and the production detector so all three embed the same way. Backbone id: a timm id loads via timm; "clip:<name>" loads via the `clip` package instead."""

import os
from pathlib import Path

import numpy as np

# Must cover embed_batch()'s chunk_size=32 range — the TRT engine builds
# once for this whole range; a batch outside it forces a slow rebuild.
TRT_MIN_BATCH = 1
TRT_OPT_BATCH = 8
TRT_MAX_BATCH = 32


class EmbeddingBackbone:
    def __init__(
        self, backbone_id: str, img_size: int | None = None, use_trt: bool = False
    ):
        self.backbone_id = backbone_id
        # None keeps timm's 518px default (1532ms->263ms/8-crop at 224px on
        # an Orin) — only override once that accuracy trade-off is re-benchmarked.
        self.img_size = img_size
        self.is_clip = backbone_id.startswith("clip:")
        # Same onnxruntime/TensorRT pattern face_recognition.py uses — CLIP
        # isn't supported (exporting its image tower to ONNX is its own project), falls back to plain PyTorch.
        self.use_trt = use_trt and not self.is_clip
        self._model = None
        self._transform = None
        self._torch = None
        self._device = None
        self._input_size = None
        self._session = None
        self._input_name = None

    def load(self) -> "EmbeddingBackbone":
        import torch

        self._torch = torch
        # Same crop, ~40s vs ~200ms per frame on a Jetson Orin — CUDA is
        # available but nothing runs on it unless explicitly moved there.
        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.is_clip:
            import clip

            clip_name = self.backbone_id.split("clip:", 1)[1]
            self._model, self._transform = clip.load(clip_name, device=self._device)
        else:
            # Same cache fetch_models.py's fetch_hf_models() downloads into —
            # must match, or a fresh container finds nothing offline (setdefault: don't clobber a launcher-set value).
            cache_dir = Path(
                os.environ.get("TENSORRT_CACHE_DIR", "/workspace/trt_cache")
            )
            os.environ.setdefault("HF_HOME", str(cache_dir / "hf_cache"))
            import timm

            model_kwargs = {"pretrained": True, "num_classes": 0}
            cfg_overrides = {}
            if self.img_size:
                model_kwargs["img_size"] = self.img_size
                cfg_overrides["input_size"] = (3, self.img_size, self.img_size)
            self._model = timm.create_model(self.backbone_id, **model_kwargs)
            cfg = timm.data.resolve_data_config(cfg_overrides, model=self._model)
            self._transform = timm.data.create_transform(**cfg)
            self._input_size = cfg["input_size"][-1]  # square inputs only
            self._model.to(self._device)
        self._model.eval()

        if self.use_trt:
            self._load_trt_session()
        return self

    def _onnx_path(self) -> Path:
        cache_dir = Path(os.environ.get("TENSORRT_CACHE_DIR", "/workspace/trt_cache"))
        cache_dir.mkdir(parents=True, exist_ok=True)
        safe_name = self.backbone_id.replace("/", "_")
        return cache_dir / f"{safe_name}_{self._input_size}px.onnx"

    def _export_onnx(self, onnx_path: Path):
        print(
            f"[backbone] exporting {self.backbone_id} to ONNX ({self._input_size}px) -> {onnx_path} ..."
        )
        dummy = self._torch.randn(
            1, 3, self._input_size, self._input_size, device=self._device
        )
        export_kwargs = dict(
            input_names=["input"],
            output_names=["embedding"],
            dynamic_axes={"input": {0: "batch"}, "embedding": {0: "batch"}},
            opset_version=17,
        )
        try:
            # torch>=2.6's "dynamo" exporter needs the optional `onnxscript`
            # package — force the older TorchScript-based exporter instead, plenty for a plain ViT.
            self._torch.onnx.export(
                self._model, dummy, str(onnx_path), dynamo=False, **export_kwargs
            )
        except TypeError:
            self._torch.onnx.export(self._model, dummy, str(onnx_path), **export_kwargs)
        print(f"[backbone] ONNX export done: {onnx_path}")

    def _load_trt_session(self):
        import onnxruntime as ort

        onnx_path = self._onnx_path()
        if not onnx_path.exists():
            self._export_onnx(onnx_path)

        available = ort.get_available_providers()
        if (
            "TensorrtExecutionProvider" not in available
            and "CUDAExecutionProvider" not in available
        ):
            print(
                f"[backbone] no GPU execution provider available ({available}); falling back to PyTorch"
            )
            self.use_trt = False
            return

        cache_dir = str(
            Path(os.environ.get("TENSORRT_CACHE_DIR", "/workspace/trt_cache"))
        )
        providers = []
        if "TensorrtExecutionProvider" in available:
            min_shape = f"input:{TRT_MIN_BATCH}x3x{self._input_size}x{self._input_size}"
            opt_shape = f"input:{TRT_OPT_BATCH}x3x{self._input_size}x{self._input_size}"
            max_shape = f"input:{TRT_MAX_BATCH}x3x{self._input_size}x{self._input_size}"
            providers.append(
                (
                    "TensorrtExecutionProvider",
                    {
                        "trt_engine_cache_enable": True,
                        "trt_engine_cache_path": cache_dir,
                        "trt_fp16_enable": True,
                        "trt_profile_min_shapes": min_shape,
                        "trt_profile_opt_shapes": opt_shape,
                        "trt_profile_max_shapes": max_shape,
                    },
                )
            )
        if "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")

        print(
            "[backbone] loading TensorRT session (first run builds+caches the engine, can take a few min) ..."
        )
        self._session = ort.InferenceSession(str(onnx_path), providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        print(
            f"[backbone] TensorRT session ready, providers in use: {self._session.get_providers()}"
        )

    def embed_batch(self, crops: list, chunk_size: int = 32) -> np.ndarray:
        """crops: list of PIL.Image (RGB). Returns [N, D] float32, NOT normalized
        (kept raw so callers can cache before choosing a threshold strategy) — chunked internally so a large batch doesn't allocate one huge tensor."""
        if self._model is None:
            self.load()

        if self.use_trt and self._session is not None:
            return self._embed_batch_trt(crops, chunk_size)

        all_feats = []
        for i in range(0, len(crops), chunk_size):
            chunk = crops[i : i + chunk_size]
            tensors = self._torch.stack([self._transform(c) for c in chunk]).to(
                self._device
            )
            with self._torch.no_grad():
                feats = (
                    self._model.encode_image(tensors)
                    if self.is_clip
                    else self._model(tensors)
                )
            all_feats.append(feats.cpu().numpy().astype(np.float32))
        return np.concatenate(all_feats, axis=0)

    def _embed_batch_trt(self, crops: list, chunk_size: int) -> np.ndarray:
        all_feats = []
        for i in range(0, len(crops), chunk_size):
            chunk = crops[i : i + chunk_size]
            tensors = self._torch.stack([self._transform(c) for c in chunk])
            inputs = tensors.numpy().astype(np.float32)
            (feats,) = self._session.run(None, {self._input_name: inputs})
            all_feats.append(feats.astype(np.float32))
        return np.concatenate(all_feats, axis=0)
