"""
Minimal inference-only EfficientAT MobileNetV3 (``mn10_as``) audio tagger.

Vendored and trimmed from https://github.com/fschmid56/EfficientAT
(MIT License, Copyright (c) 2022 Florian Schmid). Only what inference with the
AudioSet checkpoints needs is kept, and the torchvision / torchaudio
dependencies are replaced with plain torch so it runs in the hri image as-is.
Module names and order match the original exactly, so the released state dicts
load with ``strict=True``.

Input: mono float waveform at 32 kHz in [-1, 1]. Output: 527 AudioSet logits
(apply sigmoid) plus a 960-d embedding.
"""

from __future__ import annotations

from functools import partial
from typing import List, Optional, Tuple

import torch
import torch.nn.functional as F
from torch import Tensor, nn

SAMPLE_RATE = 32000
WEIGHTS_URL = (
    "https://github.com/fschmid56/EfficientAT/releases/download/v0.0.1/"
    "mn10_as_mAP_471.pt"
)
WEIGHTS_FILE = "mn10_as_mAP_471.pt"


# ── frontend ─────────────────────────────────────────────────────────────────


def _kaldi_mel_banks(
    n_mels: int, n_fft: int, sr: int, fmin: float, fmax: float
) -> Tensor:
    """Kaldi-style triangular mel filterbank, shape (n_mels, n_fft // 2 + 1).

    Equivalent to ``torchaudio.compliance.kaldi.get_mel_banks`` with no VTLN
    warping, padded with a zero column for the Nyquist bin (as EfficientAT does).
    """

    def mel(f):
        return 1127.0 * torch.log1p(f / 700.0)

    num_fft_bins = n_fft // 2
    mel_low, mel_high = mel(torch.tensor(fmin)), mel(torch.tensor(fmax))
    delta = (mel_high - mel_low) / (n_mels + 1)
    b = torch.arange(n_mels, dtype=torch.float64).unsqueeze(1)
    left = mel_low + b * delta
    center = left + delta
    right = center + delta
    m = mel(sr / n_fft * torch.arange(num_fft_bins, dtype=torch.float64)).unsqueeze(0)
    up = (m - left) / (center - left)
    down = (right - m) / (right - center)
    banks = torch.clamp(torch.minimum(up, down), min=0.0).float()
    return F.pad(banks, (0, 1))


class MelFrontend(nn.Module):
    """EfficientAT ``AugmentMelSTFT`` in eval mode (no augmentation)."""

    def __init__(
        self, n_mels=128, sr=SAMPLE_RATE, win_length=800, hopsize=320, n_fft=1024
    ):
        super().__init__()
        self.n_fft, self.hopsize, self.win_length = n_fft, hopsize, win_length
        fmax = sr // 2 - 1000  # EfficientAT default: sr/2 - fmax_aug_range/2
        self.register_buffer(
            "window", torch.hann_window(win_length, periodic=False), persistent=False
        )
        self.register_buffer(
            "preemphasis", torch.tensor([[[-0.97, 1.0]]]), persistent=False
        )
        self.register_buffer(
            "mel_basis",
            _kaldi_mel_banks(n_mels, n_fft, sr, 0.0, fmax),
            persistent=False,
        )

    def forward(self, x: Tensor) -> Tensor:
        x = F.conv1d(x.unsqueeze(1), self.preemphasis).squeeze(1)
        spec = torch.stft(
            x,
            self.n_fft,
            hop_length=self.hopsize,
            win_length=self.win_length,
            center=True,
            normalized=False,
            window=self.window,
            return_complex=True,
        )
        power = spec.real**2 + spec.imag**2
        melspec = torch.log(torch.matmul(self.mel_basis, power) + 1e-5)
        return (melspec + 4.5) / 5.0  # EfficientAT "fast normalization"


# ── model ────────────────────────────────────────────────────────────────────


def _make_divisible(v: float, divisor: int = 8) -> int:
    new_v = max(divisor, int(v + divisor / 2) // divisor * divisor)
    if new_v < 0.9 * v:
        new_v += divisor
    return new_v


class ConvNormActivation(nn.Sequential):
    """Same layout as ``torchvision.ops.misc.ConvNormActivation`` (conv, bn, act)."""

    def __init__(
        self,
        in_ch,
        out_ch,
        kernel_size=3,
        stride=1,
        groups=1,
        dilation=1,
        norm_layer=None,
        activation_layer=None,
    ):
        padding = (kernel_size - 1) // 2 * dilation
        layers: List[nn.Module] = [
            nn.Conv2d(
                in_ch,
                out_ch,
                kernel_size,
                stride,
                padding,
                dilation=dilation,
                groups=groups,
                bias=norm_layer is None,
            )
        ]
        if norm_layer is not None:
            layers.append(norm_layer(out_ch))
        if activation_layer is not None:
            layers.append(activation_layer(inplace=True))
        super().__init__(*layers)


class SqueezeExcitation(nn.Module):
    """Channel squeeze-excitation (EfficientAT ``se_dims='c'``)."""

    def __init__(self, input_dim: int, squeeze_dim: int):
        super().__init__()
        self.fc1 = nn.Linear(input_dim, squeeze_dim)
        self.fc2 = nn.Linear(squeeze_dim, input_dim)
        self.activation = nn.ReLU()
        self.scale_activation = nn.Sigmoid()

    def forward(self, x: Tensor) -> Tensor:
        scale = torch.mean(x, (2, 3), keepdim=True)
        shape = scale.size()
        scale = self.fc2(self.activation(self.fc1(scale.squeeze(2).squeeze(2))))
        return self.scale_activation(scale).view(shape) * x


class ConcurrentSEBlock(nn.Module):
    """Holds a single channel SE layer; kept only so state-dict keys match."""

    def __init__(self, c_dim: int, se_r: int = 4):
        super().__init__()
        self.conc_se_layers = nn.ModuleList(
            [SqueezeExcitation(c_dim, _make_divisible(c_dim // se_r, 8))]
        )

    def forward(self, x: Tensor) -> Tensor:
        return self.conc_se_layers[0](x)


class InvertedResidual(nn.Module):
    def __init__(self, cin, kernel, cexp, cout, use_se, use_hs, stride, norm_layer):
        super().__init__()
        self.use_res_connect = stride == 1 and cin == cout
        act = nn.Hardswish if use_hs else nn.ReLU
        layers: List[nn.Module] = []
        if cexp != cin:
            layers.append(
                ConvNormActivation(
                    cin, cexp, 1, norm_layer=norm_layer, activation_layer=act
                )
            )
        layers.append(
            ConvNormActivation(
                cexp,
                cexp,
                kernel,
                stride,
                groups=cexp,
                norm_layer=norm_layer,
                activation_layer=act,
            )
        )
        if use_se:
            layers.append(ConcurrentSEBlock(cexp))
        layers.append(ConvNormActivation(cexp, cout, 1, norm_layer=norm_layer))
        self.block = nn.Sequential(*layers)

    def forward(self, x: Tensor) -> Tensor:
        out = self.block(x)
        return out + x if self.use_res_connect else out


# (input, kernel, expanded, out, use_se, use_hs, stride) — MobileNetV3-Large.
_MN_CONF = [
    (16, 3, 16, 16, False, False, 1),
    (16, 3, 64, 24, False, False, 2),
    (24, 3, 72, 24, False, False, 1),
    (24, 5, 72, 40, True, False, 2),
    (40, 5, 120, 40, True, False, 1),
    (40, 5, 120, 40, True, False, 1),
    (40, 3, 240, 80, False, True, 2),
    (80, 3, 200, 80, False, True, 1),
    (80, 3, 184, 80, False, True, 1),
    (80, 3, 184, 80, False, True, 1),
    (80, 3, 480, 112, True, True, 1),
    (112, 3, 672, 112, True, True, 1),
    (112, 5, 672, 160, True, True, 2),
    (160, 5, 960, 160, True, True, 1),
    (160, 5, 960, 160, True, True, 1),
]


class MN(nn.Module):
    """EfficientAT MobileNetV3 with the ``mlp`` classification head."""

    def __init__(self, width_mult: float = 1.0, num_classes: int = 527):
        super().__init__()
        ch = partial(lambda c, w: _make_divisible(c * w, 8), w=width_mult)
        norm_layer = partial(nn.BatchNorm2d, eps=0.001, momentum=0.01)

        layers: List[nn.Module] = [
            ConvNormActivation(
                1,
                ch(_MN_CONF[0][0]),
                3,
                2,
                norm_layer=norm_layer,
                activation_layer=nn.Hardswish,
            )
        ]
        for cin, k, cexp, cout, se, hs, s in _MN_CONF:
            layers.append(
                InvertedResidual(ch(cin), k, ch(cexp), ch(cout), se, hs, s, norm_layer)
            )
        last_in = ch(_MN_CONF[-1][3])
        last_out = 6 * last_in
        layers.append(
            ConvNormActivation(
                last_in,
                last_out,
                1,
                norm_layer=norm_layer,
                activation_layer=nn.Hardswish,
            )
        )
        self.features = nn.Sequential(*layers)
        last_channel = ch(1280)
        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(start_dim=1),
            nn.Linear(last_out, last_channel),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.2, inplace=True),
            nn.Linear(last_channel, num_classes),
        )

    def forward(self, x: Tensor) -> Tuple[Tensor, Tensor]:
        """x: (batch, 1, n_mels, frames) → (logits (batch, 527), embedding (batch, C))."""
        x = self.features(x)
        embedding = F.adaptive_avg_pool2d(x, (1, 1)).flatten(1)
        return self.classifier(x), embedding


class AudioTagger(nn.Module):
    """Waveform (32 kHz) → AudioSet probabilities. Frontend + MN in one module."""

    def __init__(self, weights_path: str, device: Optional[str] = None):
        super().__init__()
        self.mel = MelFrontend()
        self.model = MN(width_mult=1.0)
        state = torch.load(weights_path, map_location="cpu", weights_only=True)
        self.model.load_state_dict(state, strict=True)
        self.device = torch.device(
            device or ("cuda" if torch.cuda.is_available() else "cpu")
        )
        self.to(self.device).eval()

    @torch.inference_mode()
    def forward(self, waveform: Tensor) -> Tuple[Tensor, Tensor]:
        """waveform: (batch, samples) float in [-1, 1] at 32 kHz.

        Returns (probabilities (batch, 527), embedding (batch, 960)) on CPU.
        """
        spec = self.mel(waveform.to(self.device))
        logits, emb = self.model(spec.unsqueeze(1))
        return torch.sigmoid(logits.float()).cpu(), emb.float().cpu()
