"""Speaker verification utilities (issue #1459).

Pure-python building blocks (no ROS) used by ``scripts/speaker_verification.py``:

- ``features``: Kaldi-compatible log-mel filterbank (numpy, torchaudio optional).
- ``embedder``: ONNX speaker-embedding model wrapper (WeSpeaker ResNet34 by default).
- ``silero``: streaming Silero VAD (ONNX, v4 and v5 graphs).
- ``segmenter``: VAD hysteresis + adaptive energy gate + speech segmentation.
- ``profiles``: enrolled speaker profiles (anchor + adaptive pool) persisted as .npz.
- ``verifier``: scoring, TARGET / NOT_TARGET / UNKNOWN decisions and enrollment.
"""
