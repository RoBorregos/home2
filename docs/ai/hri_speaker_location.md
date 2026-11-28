# HRI: who is speaking and from where (speaker id + sound direction)

Context for issue #1459 and for the next step: **fusing it with vision's
"who is talking" detection**. Read this before touching
`hri/packages/speech/scripts/speaker_verification.py`.

## Files

| File | Role |
| --- | --- |
| `hri/packages/speech/scripts/speaker_verification.py` | The ROS node (only new node) |
| `hri/packages/speech/speech/speaker_id_utils.py` | VAD, segmenter, fbank, ONNX embedder, profiles, decisions |
| `hri/packages/speech/speech/spatial_utils.py` | SRP-PHAT direction, speaker tracker, MVDR beamformer |
| `hri/packages/speech/config/speaker_verification.yaml` | Calibrated parameters |
| `hri/packages/speech/launch/devices_launch.py` | Starts the node with the HRI pipeline when the ReSpeaker is connected |
| `hri/packages/speech/scripts/audio_capturer.py` | Also publishes the 6 raw ReSpeaker channels |
| `hri/benchmarks/speaker_id/` | Record, replay, evaluate, live monitor/enrollment, tests (see its README) |
| `docker/hri/scripts/download-model.sh` | Option `speaker-id` downloads the embedding model |

## Pipeline

```
ReSpeaker (6 ch: ch0 chip beam, ch1-4 raw mics, ch5 playback ref)
  audio_capturer ──> RAW_AUDIO_TOPIC           (ch0, unchanged)
                 └─> MULTICHANNEL_AUDIO_TOPIC  (6 ch interleaved int16)
speaker_verification
  ch1-4 ─> MvdrFrontEnd (RTF-MVDR beam) ─> Silero VAD + segmenter
        ─> WeSpeaker ResNet34 ONNX embedding (GPU) ─> cosine vs profiles
        ─> TARGET / NOT_TARGET / UNKNOWN / MIXED
  ch1-4 ─> SRP-PHAT over the SAME window as each embedding
        ─> SpeakerTracker: remembers name <-> direction for 8 s
```

## Topics (`frida_constants/hri_constants.py`)

| Constant | Type | Content |
| --- | --- | --- |
| `SPEAKER_ID_LOCATED_TOPIC` | String (JSON) | **The one to fuse with vision.** Who + where, every 0.5 s while someone talks and once per finished segment |
| `SPEAKER_ID_COMMAND_TOPIC` | String (JSON) | `enroll_start`, `enroll_commit`, `enroll_lookback`, `set_target`, `forget`, `list`, `reload` |
| `SPEAKER_ID_STATUS_TOPIC` | String (JSON, latched) | Replies to commands |
| `MULTICHANNEL_AUDIO_TOPIC` | AudioData | 6-ch raw audio from `audio_capturer` |

Voice activity and the chip's own direction already exist
(`VOICE_ACTIVITY_TOPIC`, `RESPEAKER_DOA_TOPIC`); the robot's TTS state is `/saying`.

`SPEAKER_ID_LOCATED_TOPIC` message:

```json
{"speaker": "mario" | "unknown", "via": "voice" | "direction" | "none",
 "azimuth_deg": -104.0, "direction": "right", "doa_confidence": 0.6,
 "doa_reliable": true, "score": 0.56, "label": "TARGET", "final": true,
 "part": "whole" | "target" | "other", "seg_id": 16, "dur": 15.0,
 "best": "mario", "second": "", "second_score": 0.0, "snr_db": 9.0,
 "sources": [...], "tracks": [...], "t": 1795566893.2}
```

- **Azimuth frame:** 0° = robot front, +90° = robot's left, −90° = right,
  ±180° = back. `direction` is the 8-sector label. `null` without the ReSpeaker.
- `via: "direction"`: the voice was unsure but came from where that person was
  identified in the last 8 s. A clearly rejected voice is never named by direction.
- `part: "target"/"other"`: a MIXED segment (two voices) split per voice.
- Check `doa_reliable` (confidence ≥ 0.15) before trusting `azimuth_deg`.

## Measured results (HRI Orin, noisy lab)

Live 2026-11-24, talker at ~0.8 m, another person talking at the same time:

| Position | Identified | Azimuth | Error |
| --- | --- | --- | --- |
| Front (0°) | mario | 0° | 0–1° |
| Right (−90°) | mario | −104° to −106° | 14–16° |
| Left (+90°) | mario | +75° | 15° |

- 0 of 31 impostor segments accepted (max 0.30; accept 0.38, reject 0.24).
- MVDR beam vs chip ch0: genuine min score 0.46 → 0.59, d' 7.7 → 9.5.
- Decision latency ~90 ms; embedding ~13 ms on the GPU; ~37 % of one CPU core.

## Fusion with vision (next step)

- Match each located message to the person whose camera bearing is closest to
  `azimuth_deg`. Expect ±15° error; audio cannot separate two people closer than
  ~25°, let vision's mouth/talking detection break ties.
- Same robot frame on both sides. The mic offset is `DOA_OFFSET_DEG: -15`
  (calibrated); recalibrate with `hri/benchmarks/speaker_id/evaluate.py doa`
  if the ReSpeaker is remounted. Known bias: right reads ~15° too far back.

## Gotchas

- **OpenBLAS spinning:** the node sets `OPENBLAS_NUM_THREADS=1` before numpy
  loads (95 % → ~37 % of a core). Keep it at the top of new audio nodes.
- **Profiles are front-end specific:** enrolled on MVDR; re-enroll if
  `MONO_SOURCE` changes. They live in `assets/downloads/speaker_id/profiles/`
  (gitignored) and are lost if the checkout is cleaned.
- **PipeWire grabs the ReSpeaker after an Orin reboot** (container reads zeros):
  check `pactl list cards`; free the capture with profile `output:analog-stereo`
  and restore `output:analog-stereo+input:analog-surround-21` afterwards.
- `ros2 run ... -p PARAM:=""` fails to parse; use a params YAML.
