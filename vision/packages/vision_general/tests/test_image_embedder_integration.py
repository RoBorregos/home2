"""Real encoder tests; no downloads. Set CLIP_TEST_WEIGHTS for pretrained coverage."""

import os
import importlib.util

import numpy as np
import pytest

from test_image_embedder import ImageEmbedder


def check_baseline(embedder, crops, expected):
    """Optional A/B check against an exported baseline, sharing exact weights."""
    path = os.environ.get("IMAGE_EMBEDDER_BASELINE")
    if not path:
        return
    spec = importlib.util.spec_from_file_location("baseline_image_embedder", path)
    baseline_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline_module)
    baseline = baseline_module.ImageEmbedder(embedder.model_id)
    baseline.__dict__.update(embedder.__dict__)
    np.testing.assert_array_equal(baseline.embed_batch(crops), expected)


def check_clip_encoders(embedder, clip, torch, crops):
    texts = ["a red cup", "a blue box"]
    images = embedder.embed_batch(crops)
    queries = embedder.embed_text(texts)
    assert images.shape == queries.shape == (2, 512)
    assert images.dtype == queries.dtype == np.float32
    assert embedder.dim == 512
    assert not embedder._model.training
    with torch.no_grad():
        native_images = (
            embedder._model.encode_image(
                torch.stack([embedder._transform(crop) for crop in crops]).to(
                    embedder._device
                )
            )
            .float()
            .cpu()
            .numpy()
        )
        native_text = (
            embedder._model.encode_text(clip.tokenize(texts).to(embedder._device))
            .float()
            .cpu()
            .numpy()
        )
    np.testing.assert_allclose(images, native_images, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(queries, native_text, rtol=1e-5, atol=1e-5)
    check_baseline(embedder, crops, images)
    normalized_images = embedder.embed_batch(crops, normalize=True)
    normalized_text = embedder.embed_text(texts, normalize=True)
    np.testing.assert_allclose(np.linalg.norm(normalized_images, axis=1), 1, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(normalized_text, axis=1), 1, atol=1e-6)
    expected = (
        native_images / np.linalg.norm(native_images, axis=1, keepdims=True)
    ) @ (native_text / np.linalg.norm(native_text, axis=1, keepdims=True)).T
    np.testing.assert_allclose(
        normalized_images @ normalized_text.T, expected, atol=1e-5
    )
    with pytest.raises(RuntimeError, match="too long"):
        embedder.embed_text(["cup " * 100])


def test_real_clip_encoder_contract_without_weights(monkeypatch):
    torch = pytest.importorskip("torch")
    clip = pytest.importorskip("clip")
    from clip.model import CLIP
    from torchvision.transforms import ToTensor
    from PIL import Image

    # Small random model: real tokenizer/encoder/device/dtype behavior, no
    # pretrained accuracy claim. Production checkpoint is checked separately.
    model = CLIP(
        embed_dim=512,
        image_resolution=32,
        vision_layers=1,
        vision_width=64,
        vision_patch_size=16,
        context_length=77,
        vocab_size=49408,
        transformer_width=64,
        transformer_heads=1,
        transformer_layers=1,
    )
    monkeypatch.setattr(
        clip, "load", lambda name, device: (model.to(device), ToTensor())
    )
    embedder = ImageEmbedder("clip:ViT-B/32")
    check_clip_encoders(
        embedder,
        clip,
        torch,
        [Image.new("RGB", (32, 32), color) for color in ("red", "blue")],
    )


def test_real_timm_dimension_and_raw_output(monkeypatch):
    torch = pytest.importorskip("torch")
    timm = pytest.importorskip("timm")
    from PIL import Image

    create_model = timm.create_model

    def without_download(name, **kwargs):
        kwargs["pretrained"] = False
        return create_model(name, **kwargs)

    monkeypatch.setattr(timm, "create_model", without_download)
    embedder = ImageEmbedder("vit_small_patch14_dinov2.lvd142m", img_size=28).load()
    crops = [Image.new("RGB", (28, 28), "red")]
    raw = embedder.embed_batch(crops)
    assert raw.shape == (1, embedder.dim) == (1, 384)
    assert raw.dtype == np.float32
    with torch.no_grad():
        native = (
            embedder._model(
                torch.stack([embedder._transform(c) for c in crops]).to(
                    embedder._device
                )
            )
            .cpu()
            .numpy()
            .astype(np.float32)
        )
    np.testing.assert_array_equal(raw, native)
    check_baseline(embedder, crops, raw)
    assert not np.isclose(np.linalg.norm(raw), 1)
    with pytest.raises(ValueError, match="requires a CLIP"):
        embedder.embed_text(["cup"])


@pytest.mark.skipif(
    not os.environ.get("CLIP_TEST_WEIGHTS"),
    reason="Set CLIP_TEST_WEIGHTS to a local ViT-B/32 checkpoint",
)
def test_pretrained_clip_vit_b32(monkeypatch):
    torch = pytest.importorskip("torch")
    clip = pytest.importorskip("clip")
    from PIL import Image

    load = clip.load
    monkeypatch.setattr(
        clip,
        "load",
        lambda name, device: load(os.environ["CLIP_TEST_WEIGHTS"], device=device),
    )
    check_clip_encoders(
        ImageEmbedder("clip:ViT-B/32"),
        clip,
        torch,
        [Image.new("RGB", (224, 224), color) for color in ("red", "blue")],
    )
