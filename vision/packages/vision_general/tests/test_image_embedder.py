"""API regression tests without model downloads or a GPU."""

from contextlib import nullcontext
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts/utils/models/image_embedder.py"
)
spec = importlib.util.spec_from_file_location("image_embedder", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ImageEmbedder = module.ImageEmbedder


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def to(self, device):
        assert device == "cpu"
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def loaded_embedder(model_id="clip:ViT-B/32"):
    embedder = ImageEmbedder(model_id)
    embedder._model = SimpleNamespace(
        text_projection=np.empty((512, 512)), num_features=768
    )
    embedder._device = "cpu"
    embedder._torch = SimpleNamespace(no_grad=nullcontext)
    return embedder


@pytest.mark.parametrize(
    "use_trt,has_session", [(False, False), (True, True), (True, False)]
)
def test_image_defaults_preserve_raw_gallery_vectors(use_trt, has_session):
    embedder = loaded_embedder("dinov2")
    embedder.use_trt = use_trt
    embedder._session = object() if has_session else None
    raw = np.array([[3, 4], [0, 0], [5, 12]], dtype=np.float32)
    forward = Mock(side_effect=lambda indices: raw[indices])
    unused = Mock(side_effect=AssertionError("wrong backend"))
    embedder._embed_chunk_torch = unused if has_session else forward
    embedder._embed_chunk_trt = forward if has_session else unused
    np.testing.assert_array_equal(embedder.embed_batch([0, 1, 2], 2), raw)
    normalized = embedder.embed_batch([0, 1, 2], 2, normalize=True)
    np.testing.assert_allclose(normalized, [[0.6, 0.8], [0, 0], [5 / 13, 12 / 13]])
    assert normalized.dtype == np.float32
    np.testing.assert_array_equal(raw[0], [3, 4])
    unused.assert_not_called()


def test_clip_text_chunks_order_dtype_and_normalization(monkeypatch):
    embedder = loaded_embedder()
    tokenize = Mock(side_effect=lambda texts: Tensor([[int(t)] for t in texts]))
    monkeypatch.setitem(sys.modules, "clip", SimpleNamespace(tokenize=tokenize))

    def encode(tokens):
        values = np.zeros((len(tokens.values), 512), dtype=np.float16)
        values[:, 0] = tokens.values[:, 0]
        return Tensor(values)

    embedder._model.encode_text = Mock(side_effect=encode)
    texts = [str(i) for i in range(35)]
    raw = embedder.embed_text(texts)
    assert raw.shape == (35, embedder.dim)
    assert raw.dtype == np.float32
    np.testing.assert_array_equal(raw[:, 0], np.arange(35))
    assert [len(call.args[0]) for call in tokenize.call_args_list] == [32, 3]
    normalized = embedder.embed_text(texts, normalize=True)
    np.testing.assert_array_equal(normalized[0], np.zeros(512))
    np.testing.assert_allclose(np.linalg.norm(normalized[1:], axis=1), 1)


def test_timm_rejects_text_without_loading():
    embedder = ImageEmbedder("vit_base_patch14_dinov2.lvd142m")
    embedder.load = Mock(side_effect=AssertionError("should not load"))
    with pytest.raises(ValueError, match="requires a CLIP model"):
        embedder.embed_text(["a cup"])


@pytest.mark.parametrize("model_id,dim", [("clip:ViT-B/32", 512), ("dinov2", 768)])
def test_dimension_loads_once(model_id, dim):
    embedder = ImageEmbedder(model_id)
    embedder.load = Mock(
        side_effect=lambda: setattr(
            embedder, "_model", loaded_embedder(model_id)._model
        )
    )
    assert embedder.dim == dim
    assert embedder.dim == dim
    embedder.load.assert_called_once()


def test_empty_inputs_and_invalid_chunk_size():
    embedder = loaded_embedder()
    assert embedder.embed_batch([]).shape == (0, 512)
    assert embedder.embed_text([]).shape == (0, 512)
    assert embedder.embed_text([]).dtype == np.float32
    with pytest.raises(ValueError, match="positive"):
        embedder.embed_batch([object()], chunk_size=0)


def test_single_string_rejected_before_loading():
    embedder = ImageEmbedder("clip:ViT-B/32")
    embedder.load = Mock(side_effect=AssertionError("should not load"))
    with pytest.raises(TypeError, match="list of strings"):
        embedder.embed_text("a red cup")


def test_tokenizer_errors_are_preserved(monkeypatch):
    embedder = loaded_embedder()
    tokenize = Mock(side_effect=RuntimeError("Input is too long"))
    monkeypatch.setitem(sys.modules, "clip", SimpleNamespace(tokenize=tokenize))
    with pytest.raises(RuntimeError, match="too long"):
        embedder.embed_text(["a very long query"])


def test_lazy_text_loading(monkeypatch):
    embedder = ImageEmbedder("clip:ViT-B/32", use_trt=True)
    assert not embedder.use_trt

    def load():
        embedder._model = SimpleNamespace(
            encode_text=lambda tokens: tokens,
            text_projection=np.empty((512, 512)),
        )
        embedder._torch = SimpleNamespace(no_grad=nullcontext)
        embedder._device = "cpu"

    embedder.load = Mock(side_effect=load)
    monkeypatch.setitem(
        sys.modules,
        "clip",
        SimpleNamespace(tokenize=lambda texts: Tensor(np.ones((len(texts), 512)))),
    )
    assert embedder.embed_text(["cup"]).shape == (1, 512)
    assert embedder.embed_text(["box"]).shape == (1, 512)
    embedder.load.assert_called_once()
