"""Frozen image embedder shared by gallery_build.py, the benchmark and the production detector.

A timm id (e.g. ``vit_base_patch14_dinov2.lvd142m``) loads through timm; ``clip:<name>`` loads through ``clip``.
"""

import os
from pathlib import Path

import numpy as np

CLIP_PREFIX = "clip:"
DEFAULT_TENSORRT_CACHE_DIR = "/workspace/trt_cache"

# The TensorRT engine is built once for this whole batch range; a batch outside
# it forces a slow rebuild. TRT_MAX_BATCH is also embed_batch()'s chunk size.
TRT_MIN_BATCH = 1
TRT_OPT_BATCH = 8
TRT_MAX_BATCH = 32


def tensorrt_cache_dir() -> Path:
    """Persistent cache mount (TENSORRT_CACHE_DIR): engines, HF weights and the built gallery."""
    return Path(os.environ.get("TENSORRT_CACHE_DIR", DEFAULT_TENSORRT_CACHE_DIR))


class ImageEmbedder:
    """Embeds PIL crops into feature vectors with a frozen model.

    Runs through TensorRT (onnxruntime) when ``use_trt`` is set and a GPU provider
    exists, otherwise PyTorch. CLIP has no TensorRT path.

    Args:
        model_id: timm id, or ``clip:<name>`` (e.g. ``clip:ViT-B/32``).
        img_size: Input size override; None keeps timm's default (518px for DINOv2).
        use_trt: Run through TensorRT. Ignored for CLIP.
    """

    def __init__(
        self, model_id: str, img_size: int | None = None, use_trt: bool = False
    ):
        self.model_id = model_id
        self.img_size = img_size
        self.is_clip = model_id.startswith(CLIP_PREFIX)
        self.use_trt = use_trt and not self.is_clip
        self._model = None
        self._transform = None
        self._torch = None
        self._device = None
        self._input_size = None
        self._session = None
        self._input_name = None

    def load(self) -> "ImageEmbedder":
        """Loads the weights (and the TensorRT session if enabled). Returns self."""
        import torch

        self._torch = torch
        # Nothing runs on the GPU unless moved there (~40 s vs ~200 ms per
        # frame on an Orin).
        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.is_clip:
            self._load_clip()
        else:
            self._load_timm()
        self._model.eval()

        if self.use_trt:
            self._load_trt_session()
        return self

    def embed_batch(self, crops: list, chunk_size: int = TRT_MAX_BATCH) -> np.ndarray:
        """Embeds RGB PIL crops.

        Args:
            crops: Non-empty list of PIL images.
            chunk_size: Crops per forward pass, so a large batch does not
                allocate one huge tensor.

        Returns:
            ``[N, D]`` float32 array, not normalized: callers cache the raw
            vectors before choosing a threshold strategy.
        """
        if self._model is None:
            self.load()

        embed_chunk = (
            self._embed_chunk_trt
            if self.use_trt and self._session is not None
            else self._embed_chunk_torch
        )
        feats = [
            embed_chunk(crops[i : i + chunk_size])
            for i in range(0, len(crops), chunk_size)
        ]
        return np.concatenate(feats, axis=0)

    def _load_clip(self) -> None:
        import clip

        name = self.model_id.removeprefix(CLIP_PREFIX)
        self._model, self._transform = clip.load(name, device=self._device)

    def _load_timm(self) -> None:
        # Same cache fetch_models.py downloads into, or a fresh container finds
        # nothing offline. setdefault keeps a launcher-set value.
        os.environ.setdefault("HF_HOME", str(tensorrt_cache_dir() / "hf_cache"))
        import timm

        model_kwargs = {"pretrained": True, "num_classes": 0}
        cfg_overrides = {}
        if self.img_size:
            model_kwargs["img_size"] = self.img_size
            cfg_overrides["input_size"] = (3, self.img_size, self.img_size)
        self._model = timm.create_model(self.model_id, **model_kwargs)
        cfg = timm.data.resolve_data_config(cfg_overrides, model=self._model)
        self._transform = timm.data.create_transform(**cfg)
        self._input_size = cfg["input_size"][-1]  # square inputs only
        self._model.to(self._device)

    def _embed_chunk_torch(self, chunk: list) -> np.ndarray:
        tensors = self._torch.stack([self._transform(c) for c in chunk]).to(
            self._device
        )
        with self._torch.no_grad():
            feats = (
                self._model.encode_image(tensors)
                if self.is_clip
                else self._model(tensors)
            )
        return feats.cpu().numpy().astype(np.float32)

    def _embed_chunk_trt(self, chunk: list) -> np.ndarray:
        tensors = self._torch.stack([self._transform(c) for c in chunk])
        inputs = tensors.numpy().astype(np.float32)
        (feats,) = self._session.run(None, {self._input_name: inputs})
        return feats.astype(np.float32)

    def _onnx_path(self) -> Path:
        cache_dir = tensorrt_cache_dir()
        cache_dir.mkdir(parents=True, exist_ok=True)
        safe_name = self.model_id.replace("/", "_")
        return cache_dir / f"{safe_name}_{self._input_size}px.onnx"

    def _export_onnx(self, onnx_path: Path) -> None:
        print(
            f"[image_embedder] exporting {self.model_id} to ONNX "
            f"({self._input_size}px) -> {onnx_path} ..."
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
            # torch>=2.6's dynamo exporter needs the optional `onnxscript`
            # package; the TorchScript exporter is plenty for a plain ViT.
            self._torch.onnx.export(
                self._model, dummy, str(onnx_path), dynamo=False, **export_kwargs
            )
        except TypeError:
            self._torch.onnx.export(self._model, dummy, str(onnx_path), **export_kwargs)
        print(f"[image_embedder] ONNX export done: {onnx_path}")

    def _trt_provider(self) -> tuple:
        shape = f"3x{self._input_size}x{self._input_size}"
        return (
            "TensorrtExecutionProvider",
            {
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": str(tensorrt_cache_dir()),
                "trt_fp16_enable": True,
                "trt_profile_min_shapes": f"input:{TRT_MIN_BATCH}x{shape}",
                "trt_profile_opt_shapes": f"input:{TRT_OPT_BATCH}x{shape}",
                "trt_profile_max_shapes": f"input:{TRT_MAX_BATCH}x{shape}",
            },
        )

    def _load_trt_session(self) -> None:
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
                f"[image_embedder] no GPU execution provider ({available}); "
                "falling back to PyTorch"
            )
            self.use_trt = False
            return

        providers = []
        if "TensorrtExecutionProvider" in available:
            providers.append(self._trt_provider())
        if "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")

        print(
            "[image_embedder] loading TensorRT session "
            "(first run builds and caches the engine, can take a few min) ..."
        )
        self._session = ort.InferenceSession(str(onnx_path), providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        print(
            "[image_embedder] TensorRT session ready, providers in use: "
            f"{self._session.get_providers()}"
        )
