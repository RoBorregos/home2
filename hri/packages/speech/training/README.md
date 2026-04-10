# Wakeword retraining pipeline

End-to-end pipeline to record, augment, and train custom openwakeword
classifier heads for the `yes`, `no`, `stop`, and `frida` commands used by
the HRI ask/confirm task. The trained `.onnx` files are drop-in compatible
with `hri/packages/speech/scripts/kws_oww.py` — no inference code changes.

```
training/
├── scripts/
│   ├── record_samples.py   # ROS2 recorder → /hri/processedAudioChunk
│   ├── augment_clips.py    # raw → ~200 variants per clip
│   └── train_word.py       # features → tiny Conv1D → ONNX
├── configs/
│   └── negatives_prompts.txt   # things to read aloud for the negative set
├── data/
│   ├── raw/                # your real recordings, one folder per label
│   ├── augmented/          # produced by augment_clips.py
│   └── backgrounds/        # drop ambient noise .wav files here (optional)
└── requirements.txt
```

Trained models land in `hri/packages/speech/assets/oww_retrained/`, which is
installed by the existing `CMakeLists.txt` (it picks up every `assets/**/*.onnx`).

---

## Why this works with the existing `kws_oww.py` unchanged

`kws_oww.py:66` loads every `.onnx` file found under `model_path`. The trained
file has input shape `(1, 16, 96)` — exactly what `openwakeword.Model` feeds
into wakeword heads — so it is byte-compatible with the runtime. Feature
extraction at training time uses the same `melspectrogram.onnx` +
`embedding_model.onnx` files already sitting in
`hri/packages/speech/assets/downloads/`, so train-time and inference-time
preprocessing are identical.

---

## One-time setup

1. Install the extra deps (only `torch` if you already installed the HRI
   speech requirements):

   ```bash
   pip install -r hri/packages/speech/training/requirements.txt
   ```

2. Make sure the feature models are downloaded. The easiest way is to just
   launch `kws_oww.py` once — its `download_models()` method writes
   `melspectrogram.onnx` and `embedding_model.onnx` into
   `hri/packages/speech/assets/downloads/`.

3. (Optional but recommended) Drop a handful of ambient-noise WAVs into
   `training/data/backgrounds/`. These are used during augmentation to mix
   realistic background into every clip. Good sources:
   - A recording of the robot idling with fans and servos running
   - A recording of the lab / arena with people talking in the background
   - ~30–60 seconds each, mono 16 kHz WAV

   You can produce these yourself with `record_samples.py --word backgrounds`,
   then move the resulting files from `data/raw/backgrounds/` to
   `data/backgrounds/`.

---

## Step 1 — Record on the Orin via the ReSpeaker

The recorder subscribes to `/hri/processedAudioChunk`, i.e. the
DeepFilterNet-cleaned output of your ReSpeaker. **Launch the HRI audio stack
first** so that topic is live:

```bash
# On the Orin, in one terminal:
ros2 launch speech devices_launch.py
```

Then, in another terminal, record each word. You only need a few real clips
per word — the augmentation step turns each one into ~200 variants:

```bash
# ~10 clips each; say the word with varying intonation, distance, speed
ros2 run speech record_samples.py --word yes   --count 10 --duration 1.5
ros2 run speech record_samples.py --word no    --count 10 --duration 1.5
ros2 run speech record_samples.py --word stop  --count 10 --duration 1.5
ros2 run speech record_samples.py --word frida --count 10 --duration 1.5
```

The script prompts you, counts down `3 … 2 … 1 …` and records a 1.5-second
window. If a clip comes out too quiet it warns you so you can redo it.

### Record negatives (important — this is where most models fail)

Negatives teach the model what is **not** the wakeword. Without them, the
classifier will false-trigger on normal speech. Record some:

```bash
ros2 run speech record_samples.py --word negatives --count 30 --duration 2.0
```

While recording, read lines from
`hri/packages/speech/training/configs/negatives_prompts.txt`. It contains:

- Phonetic neighbours (`yesterday`, `nope`, `stopped`, `friday`, …)
- Normal conversational sentences
- Sentences that contain the target words in non-command context

A good negative set is the single biggest quality lever. Aim for 2–3 minutes
of total audio.

### Record backgrounds (optional but strongly recommended)

```bash
# One long 2-minute clip of whatever noise the robot actually hears:
ros2 run speech record_samples.py --word backgrounds --count 1 --duration 120 --no-prompt
# Then move the file so it is used for augmentation, not as training data:
mv hri/packages/speech/training/data/raw/backgrounds/*.wav \
   hri/packages/speech/training/data/backgrounds/
```

---

## Step 2 — Augment

Expand each raw clip into many variants (pitch shift, time stretch, gain,
random silence padding, background mixing, additive noise):

```bash
cd hri/packages/speech/training
python3 scripts/augment_clips.py --all --variants 200
```

With 10 raw clips per word × 200 variants, you get 2,000 positive samples
per word. The negatives folder gets the same treatment. If you recorded a
2-minute negatives clip, the augmenter additionally cuts random 1.28-second
windows out of it, which multiplies the effective negative count.

---

## Step 3 — Train one model per word

```bash
cd hri/packages/speech/training
python3 scripts/train_word.py --word yes   --epochs 30
python3 scripts/train_word.py --word no    --epochs 30
python3 scripts/train_word.py --word stop  --epochs 30
python3 scripts/train_word.py --word frida --epochs 30
```

Each run:

1. Builds positives from `augmented/<word>/*.wav`
2. Builds negatives from **every other** `augmented/<folder>/*.wav` (so when
   training `yes`, the clips for `no`, `stop`, `frida`, `negatives`, and
   `backgrounds` all become negatives — free hard negatives).
3. Extracts `(16, 96)` feature tensors using the openwakeword feature ONNX
   models.
4. Trains a tiny Conv1D classifier (~20 k parameters, seconds on CPU).
5. Exports to `hri/packages/speech/assets/oww_retrained/<word>.onnx`.

Expect validation accuracy in the 0.95–0.99 range. If it stays around 0.5,
check that positive and negative counts are both non-zero in the log.

---

## Step 4 — Use the retrained models

Two options:

### A. Drop-in replacement (simplest)

Copy the new models over the old ones and rebuild:

```bash
cp hri/packages/speech/assets/oww_retrained/*.onnx \
   hri/packages/speech/assets/oww/

# Inside the docker workspace:
colcon build --packages-select speech
ros2 launch speech devices_launch.py
```

### B. Point `kws_oww` at the new folder (safer — keeps the originals)

Edit `hri/packages/speech/config/kws_oww.yaml`:

```yaml
kws_oww:
  ros__parameters:
    # Was: /workspace/src/hri/packages/speech/assets/oww
    model_path: "/workspace/src/hri/packages/speech/assets/oww_retrained"
    inference_framework: "onnx"
    ...
    SENSITIVITY_THRESHOLD: 0.6   # tune this after your first run
```

`kws_oww.py:66` auto-loads every `.onnx` in that folder. The detection key
published on `/speech/oww` will be the filename stem (`"yes"`, `"no"`, ...).

### Tuning

After a first run, tune `SENSITIVITY_THRESHOLD` in `kws_oww.yaml`:

- Getting false triggers on normal speech → raise it (try 0.65, 0.75)
- Missing real commands → lower it (try 0.45, 0.35)
- `detection_cooldown` prevents a single utterance from firing twice in a row.

---

## Iterating

The whole pipeline is designed so you can add data and retrain in minutes:

1. Record more real clips (especially of people who gave false positives)
2. Re-run `augment_clips.py --all`
3. Re-run `train_word.py --word <the problem word>`
4. Redeploy

No need to re-augment words you haven't added data for.

---

## Troubleshooting

**"Missing feature model: ... melspectrogram.onnx"**
Launch `kws_oww.py` once so it downloads the feature models, or copy them
from another machine into `hri/packages/speech/assets/downloads/`.

**"No negative clips found"**
You need at least one non-empty subfolder under `data/augmented/` that is
not the target word. Record a `negatives` set and re-augment.

**Validation accuracy stuck near 0.5**
Either no positives or no negatives made it into the dataset. Check the
"positive / negative augmented clips" line in the training log. Usually
this means a folder was empty or filenames are not `*.wav`.

**Model triggers on silence**
You probably don't have enough background/noise negatives. Record 1–2
minutes of ambient noise, move it to `data/backgrounds/`, re-augment, retrain.

**Model triggers on the robot's own TTS voice**
Record 1–2 minutes of Frida speaking (via `say.py`), save as negatives,
re-augment, retrain.
