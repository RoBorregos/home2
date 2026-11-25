# HRI: who is speaking and from where (speaker-id + sound direction)

Context for the `feat/voice-recognition` work (issue #1459) and for the next
step: **fusing it with vision's "who is talking" detection**. Read this before
touching `hri/packages/speech/scripts/speaker_verification.py` or
`hri/packages/speech/speech/{speaker_id,spatial}/`.

## What it does

`speaker_verification` listens to the ReSpeaker 4-mic array and publishes, per
utterance, **who** spoke (enrolled name or unknown) and **from which direction**
(robot frame). If two people talk in the same segment, it publishes one message
per voice, each with its own direction.

It is **not started by the normal stack**: launch it explicitly.

```bash
# inside the HRI container, on the HRI Orin
ros2 launch speech speaker_verification_launch.py with_capture:=true
ros2 run speech who_where_monitor.py            # live colored view
ros2 run speech spkid_cli.py enroll mario --seconds 15   # enroll (talk 15-30 s)
ros2 run speech spkid_cli.py list | forget NAME | target NAME
```

Without the full HRI stack, `with_capture:=true` starts `audio_capturer`. With
the full stack running, omit it (the capturer is already up).

## Pipeline

```
ReSpeaker (6 ch: ch0 chip beam, ch1-4 raw mics, ch5 playback ref)
  audio_capturer ──> /hri/rawAudioChunk          (ch0, unchanged from main)
                 └─> /hri/rawAudioMultichannel   (all 6 ch interleaved int16, new)
speaker_verification
  ch1-4 ─> RTF-MVDR beamformer (spatial/beamformer.py) ─> mono
        ─> Silero VAD + segmenter ─> WeSpeaker ResNet34 ONNX embedding (GPU)
        ─> cosine vs profiles ─> TARGET / NOT_TARGET / UNKNOWN / MIXED
  ch1-4 ─> SRP-PHAT DOA (spatial/doa.py) over the SAME window as each embedding
        ─> SpeakerTracker (spatial/tracker.py): remembers name <-> direction
```

## Topics (constants in `frida_constants/hri_constants.py`)

| Topic | Type | Content |
| --- | --- | --- |
| `/hri/speaker_id/located` | String (JSON) | **The one to fuse with vision.** Who + where per partial (every 0.5 s while speaking) and final (end of segment) |
| `/hri/speech/doa` | String (JSON) | Live direction every 0.25 s while anyone speaks (no identity) |
| `/hri/speaker_id/target_active` | Bool | The target person is speaking now |
| `/hri/speaker_id/result` | String (JSON) | Raw identity decision per window/segment |
| `/hri/speaker_id/command` / `status` | String (JSON) | Enroll / forget / set target (use `spkid_cli.py`) |
| `/hri/rawAudioMultichannel` | AudioData | 6-ch raw audio from `audio_capturer` |

`/hri/speaker_id/located` fields:

```json
{"speaker": "mario" | "unknown", "via": "voice" | "direction" | "none",
 "azimuth_deg": -104.0, "direction": "right", "doa_confidence": 0.6,
 "doa_reliable": true, "score": 0.56, "label": "TARGET", "final": true,
 "part": "whole" | "target" | "other", "seg_id": 16, "dur": 15.0,
 "sources": [...], "tracks": [...], "t": 1795566893.2}
```

- **Azimuth frame:** 0° = robot front, +90° = robot's left, −90° = robot's right,
  ±180° = back. `direction` is the 8-sector label (front, front-left, left, …).
- `via: "direction"` means the voice score was unsure but the sound came from
  where that person was identified in the last 8 s. A clearly rejected voice is
  never named by direction.
- `part: "target"/"other"` appears when a MIXED segment is split per voice.
- Use `doa_reliable` (confidence ≥ 0.15) before trusting `azimuth_deg`.

## Measured results

Live in the lab, 2026-11-24, user at ~0.8 m, another person talking at the same time:

| Position | Identified | Azimuth | Error |
| --- | --- | --- | --- |
| Front (0°) | mario | 0° | 0–1° |
| Right (−90°) | mario | −104° to −106° | 14–16° |
| Left (+90°) | mario | +75° | 15° |

- Impostors: 0 of 31 segments accepted (max score 0.30; accept threshold 0.38, reject 0.24).
- On the MVDR beam vs chip ch0: genuine min score 0.46 → 0.59, d' 7.7 → 9.5.
- Decision latency ~90 ms after end of speech; embedding ~13 ms on GPU.
- CPU on the HRI Orin: ~37 % of one core for `speaker_verification` (~3 % of
  the 12 cores). `audio_capturer` with the 6-ch topic: +0.1 % vs main. For
  reference, `noise_cancellation` (main) uses ~60 % of one core.

## Fusion with vision (next step, separate branch)

- Match each `located` message to the person whose bearing from the camera is
  closest to `azimuth_deg`. Expect ±15° error; audio cannot separate two
  people closer than ~25° apart, so let vision's mouth/talking detection break ties.
- The camera bearing and the mic azimuth must be in the same robot frame. The
  mic offset is calibrated in `config/speaker_verification.yaml`
  (`DOA_OFFSET_DEG: -15`, `DOA_MIRROR: false`). If the array is remounted or
  rotated, recalibrate this offset.
- Known bias: the right side reads ~15° too far back. Fine for left/right/front
  decisions, worth a recalibration before fine angle matching.

## Key config (`hri/packages/speech/config/speaker_verification.yaml`)

| Param | Value | Meaning |
| --- | --- | --- |
| `MONO_SOURCE` | `mvdr` | Embeddings on the beamformer output (`channel` = use `MONO_CHANNEL`) |
| `ACCEPT_TH` / `REJECT_TH` | 0.38 / 0.24 | Score to name a person / to reject |
| `DOA_OFFSET_DEG` | -15 | Array-to-robot rotation (calibrated) |
| `DOA_MIN_CONF` | 0.15 | Below this, direction is marked unreliable |
| `TRACK_MAX_AGE_SEC` / `TRACK_MATCH_DEG` | 8 / 25 | Direction-based attribution window |

Profiles live in `hri/packages/speech/assets/downloads/speaker_id/profiles/`
(gitignored). They are front-end specific: a profile enrolled on ch0 scores
lower on MVDR. Re-enroll after changing `MONO_SOURCE`.

## Optional: new noise suppressor (`spatial_audio`)

`scripts/spatial_audio.py` = MVDR + streaming DeepFilterNet3 (atten 6 dB).
**Off by default.** `HRI_AUDIO_FRONTEND=spatial` makes `devices_launch.py` use
it instead of `noise_cancellation` on the processed topic. SNR vs a raw mic
+6–8 dB, but vs the current chain (ch0 + DFN) it is mixed (better in 1 of 4
recordings). **Measure Whisper WER before enabling it.**

## Testing and debug tools

- Unit tests: `hri/packages/speech/test/test_{doa,beamformer,denoise,frontend,tracker,speaker_id}.py`.
- `debug/doa/live_session.py --manual`: guided front/right/left live test.
  Use `--manual` (waits for ENTER per position); people miss countdowns.
- `debug/doa/record_doa.py` (6-ch recording), `replay_multichannel.py` (replay a
  recording as if it were the mic), `eval_frontend.py` (compare front ends).

## Gotchas (cost us time before)

- **OpenBLAS thread spinning:** numpy's OpenBLAS spins 12 threads after every
  small matrix product. The nodes set `OPENBLAS_NUM_THREADS=1` before importing
  numpy (95 % → ~37 % of a core). Keep that line at the top of new audio nodes.
- **PipeWire grabs the ReSpeaker after an Orin reboot:** the container then reads
  all zeros. Check with `pactl list cards`. Workaround: set the ReSpeaker card to
  `output:analog-stereo` to free the capture, then restore it to
  `output:analog-stereo+input:analog-surround-21` when done (teammates need it).
  Also check for a stale `audio_capturer` still holding `pcmC0D0c`.
- **Shared Orin checkout:** teammates switch `~/dev/hri` branches and clean
  untracked files. Keep recordings and profiles backed up off the robot. Files
  a script writes to `/tmp` inside the container are not on the host.
- `ros2 run ... -p PARAM:=""` fails to parse. Use the launch `params:=<yaml>` override.
