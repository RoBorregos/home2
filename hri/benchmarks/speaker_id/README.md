# Speaker id + direction benchmark

Tools to measure and debug `speaker_verification` (who is speaking and from
where, issue #1459). Nothing here runs on the robot by default. Run them inside
the HRI container with the workspace sourced.

| Tool | What for |
| --- | --- |
| `record.py` | Record the 6 ReSpeaker channels (+ chip DOA) to WAV. Stop `audio_capturer` first |
| `replay.py` | Publish a recording as if it came from the mic, to test the node without talking |
| `evaluate.py speaker` | Genuine vs impostor separation per input (chip ch0 / MVDR) and per model |
| `evaluate.py doa` | Azimuth of a recording, to calibrate `DOA_OFFSET_DEG` |
| `monitor.py` | Live who/where view, enrollment (`enroll`, `lookback`, `list`, `forget`, `target`) and the guided `session` test |
| `test_speaker_id.py` | Synthetic unit tests (no ROS, no models): `python3 -m pytest hri/benchmarks/speaker_id -q` |

```bash
python3 monitor.py enroll mario --seconds 20       # with the HRI pipeline running
python3 monitor.py session --positions front,right,left
python3 record.py 40 /tmp/mario 5 && python3 record.py 120 /tmp/others 5
python3 evaluate.py speaker /tmp/mario.wav /tmp/others.wav --models
```

Recordings and candidate models live in the gitignored
`hri/packages/speech/assets/downloads/speaker_id/{recordings,candidates}`.

## Results (HRI Orin, noisy lab)

### Model and input (`evaluate.py speaker --models`, 10 genuine / 47 impostor segments)

| Model | Input | Genuine min | Impostor max | d' | ms / 3 s segment |
| --- | --- | --- | --- | --- | --- |
| **ResNet34-LM (used)** | chip ch0 | 0.46 | 0.17 | 7.72 | 42 |
| **ResNet34-LM (used)** | **MVDR** | **0.59** | 0.23 | **9.49** | **57** |
| CAM++-LM | MVDR | 0.58 | 0.33 | 7.72 | 278 |
| ECAPA1024-LM | MVDR | 0.55 | 0.25 | 6.78 | 48 |
| ResNet152-LM | MVDR | 0.65 | 0.21 | 9.87 | 167 |
| ResNet293-LM | MVDR | 0.63 | 0.20 | 10.11 | 306 |

ResNet34 on the MVDR beam: CAM++/ECAPA separate worse, ResNet152/293 only
slightly better at 3-5x the latency.

### End to end (`replay.py`, refactored node, same recordings)

- Enrolled person: 8 of 8 segments identified, all from the front (−21° to +1°).
- Impostors: 0 of 31 segments accepted (max score 0.25; accept 0.38).

### Live (`monitor.py session`, talker at ~0.8 m, another person talking)

| Position | Identified | Azimuth | Error |
| --- | --- | --- | --- |
| Front (0°) | yes | 0° | 0–1° |
| Right (−90°) | yes | −104° to −106° | 14–16° |
| Left (+90°) | yes | +75° | 15° |

Cost: ~37 % of one CPU core and ~10 % GPU; `audio_capturer` +0.1 % for the
6-channel topic.
