"""Loads a frozen embedding backbone and embeds a batch of image crops.

Shared by gallery_build.py, the offline benchmark (vision/benchmarks/embedding_gallery/),
and the production "embedding" detector — one place that knows how to turn a
crop into a vector, so all three always embed the same way.

Backbone id convention: a timm model id (DINOv2, e.g.
"vit_small_patch14_dinov2.lvd142m") loads via timm; a "clip:<name>" id
(e.g. "clip:ViT-B/32") loads via the `clip` package instead.
"""

import numpy as np


class EmbeddingBackbone:
    def __init__(self, backbone_id: str, img_size: int | None = None):
        self.backbone_id = backbone_id
        # DINOv2's timm default is 518px — ~5.8x the FLOPs of 224px for
        # roughly the same self-attention cost per patch (measured on a
        # Jetson Orin: 1532ms -> 263ms for an 8-crop batch). None keeps
        # timm's own default; only override once the accuracy trade-off at
        # a smaller size has been re-benchmarked (see report.py).
        self.img_size = img_size
        self.is_clip = backbone_id.startswith("clip:")
        self._model = None
        self._transform = None
        self._torch = None
        self._device = None

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
            import timm

            model_kwargs = {"pretrained": True, "num_classes": 0}
            cfg_overrides = {}
            if self.img_size:
                model_kwargs["img_size"] = self.img_size
                cfg_overrides["input_size"] = (3, self.img_size, self.img_size)
            self._model = timm.create_model(self.backbone_id, **model_kwargs)
            cfg = timm.data.resolve_data_config(cfg_overrides, model=self._model)
            self._transform = timm.data.create_transform(**cfg)
            self._model.to(self._device)
        self._model.eval()
        return self

    def embed_batch(self, crops: list, chunk_size: int = 32) -> np.ndarray:
        """crops: list of PIL.Image (RGB). Returns [N, D] float32, NOT normalized
        (callers that need cosine similarity should L2-normalize, e.g. via
        gallery_matcher.l2_normalize) — kept raw here so callers that want to
        cache embeddings before deciding on a normalization/threshold strategy
        aren't forced to redo the forward pass.

        Chunked internally: a single 600+-crop batch on CPU (the offline
        benchmark's enrollment set) would otherwise allocate one huge tensor.
        """
        if self._model is None:
            self.load()
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
