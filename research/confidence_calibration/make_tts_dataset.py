#!/usr/bin/env python3

"""
TTS proxy dataset for the ask_and_confirm calibration study (#1384).

Real Whisper on synthetic speech: each edge-tts English voice (16 accents) is one "speaker" with a
fixed condition (noise type, SNR, distance). The utterance is rendered, degraded (reverb + attenuation
for distance, music / crowd / fan noise) and transcribed by faster-whisper with the robot settings
(distil-large-v3, language=en, VAD, word timestamps, hotwords and initial prompt of the HRIC dialog).
The extraction LLM is emulated by rules: carrier phrases are stripped and aliases are normalized
("coke" -> "Coca-Cola"), which is the case behind the normalization bug.

The collection protocol is the same as collect_confidence_session.py: always confirm, the intended
value is known, and a slot is retried (up to 3 attempts) until the value is right. The records have the
format of HRITasks.ask_and_confirm with "data_source": "tts".

The word probabilities are real Whisper outputs, but TTS voices are cleaner and more regular than
people, so the numbers are a proxy until the week-2 recordings exist.

    python make_tts_dataset.py --out data/tts                # needs a GPU or ~30 min on CPU
    python run_all.py --data data/tts
"""

import argparse
import asyncio
import glob
import hashlib
import json
import math
import os
import random
import re
import site
import sys

import numpy as np

from common import REPO, cp, same_value

SR = 16000
HOT_DRINKS = "fanta, water, lipton, coca-cola, soda, lemonade, pepsi, orange juice, milk, sidral mundet, iced tea, coke, red bull, soju"
DRINKS = [d.strip() for d in HOT_DRINKS.split(",") if d.strip() != "coke"]
OUT_OF_LIST_NAMES = [
    "Mohammed",
    "Ximena",
    "Siddharth",
    "Guadalupe",
    "Oluwaseun",
    "Bjorn",
]
OUT_OF_LIST_DRINKS = ["kombucha", "horchata", "sparkling water", "ginger ale"]
SPOKEN_ALIASES = {"coca-cola": "coke", "sidral mundet": "sidral", "orange juice": "OJ"}
# What the extraction LLM returns for an alias it recognizes
LLM_NORMALIZATION = {
    "coke": "Coca-Cola",
    "sidral": "Sidral Mundet",
    "oj": "orange juice",
}

TEMPLATES = {
    "name": [
        "My name is {}.",
        "I'm {}.",
        "{}.",
        "Hi, I am {}.",
        "Call me {}.",
        "Hello, my name is {}, nice to meet you.",
    ],
    "drink": [
        "I'd like a {}.",
        "My favorite drink is {}.",
        "{}, please.",
        "{}.",
        "I really like {}.",
        "Can I have some {}?",
    ],
}
NAME_CARRIERS = re.compile(
    r"^(hi|hello|hey)?\s*(my name is|my name's|i am|i'm|im|call me|it's|it is|this is)?\s*"
)
DRINK_CARRIERS = re.compile(
    r"^(i'd like|i would like|i really like|i like|i want|can i have|could i have|"
    r"my favorite drink is|my favourite drink is|i'll have)\s*"
)
ARTICLES = re.compile(r"^(a glass of|a can of|a|an|some)\s+")
TRAILERS = re.compile(r"\b(please|thanks|thank you|nice to meet you)\b.*$")

CONDITIONS = {
    "noise": ["quiet", "music", "crowd", "fan"],
    "snr": [20, 10, 5],
    "distance": [0.5, 1.5, 3.0],
}
RT60 = {0.5: 0.25, 1.5: 0.5, 3.0: 0.8}


def add_cuda_dlls():
    """pip-installed cuBLAS / cuDNN are not on the DLL path on Windows."""
    for sp in site.getsitepackages():
        for d in glob.glob(os.path.join(sp, "nvidia", "*", "bin")):
            os.add_dll_directory(d)
            os.environ["PATH"] = d + os.pathsep + os.environ["PATH"]


def load_audio(path):
    import soundfile as sf
    from scipy.signal import resample_poly

    audio, sr = sf.read(path, dtype="float32")
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    g = math.gcd(sr, SR)
    return resample_poly(audio, SR // g, sr // g).astype(np.float32)


# --- Acoustic degradation ---


def music(rng, n):
    t = np.arange(n) / SR
    out = np.zeros(n)
    chords = [[220, 277, 330], [196, 247, 294], [175, 220, 262], [165, 208, 247]]
    beat = 0.5
    for i, start in enumerate(np.arange(0, n / SR, 4 * beat)):
        mask = (t >= start) & (t < start + 4 * beat)
        for f in chords[i % 4]:
            for h, a in ((1, 1.0), (2, 0.4), (3, 0.2)):
                out[mask] += a * np.sin(2 * np.pi * f * h * t[mask] + rng.uniform(0, 6))
    kick = np.exp(-((t % beat) * 30)) * np.sin(2 * np.pi * 60 * t)
    return out + 2 * kick


def fan(rng, n):
    white = rng.standard_normal(n)
    return np.convolve(white, np.ones(8) / 8, mode="same")  # low-passed hum


def crowd(rng, n, babble_pool):
    out = np.zeros(n)
    for _ in range(6):
        clip = babble_pool[rng.integers(len(babble_pool))]
        offset = rng.integers(len(clip))
        loop = np.tile(clip, n // len(clip) + 2)[offset : offset + n]
        out += loop * rng.uniform(0.5, 1.0)
    return out


def reverb(rng, speech, distance):
    rt60 = RT60[distance]
    n = int(rt60 * SR)
    tail = rng.standard_normal(n) * np.exp(-6.9 * np.arange(n) / n)
    rir = tail * 0.4 * (distance / 3.0)
    rir[0] = 1.0
    wet = np.convolve(speech, rir)[: len(speech) + n]
    return wet / (np.abs(wet).max() + 1e-9)


def degrade(rng, speech, cond, babble_pool):
    speech = reverb(rng, speech, cond["distance"])
    pad = np.zeros(int(0.6 * SR))
    speech = np.concatenate([pad, speech, pad])
    n = len(speech)
    # Farther speakers are quieter relative to the room noise (half the free-field loss)
    snr = cond["snr"] - 10 * math.log10(cond["distance"] / 0.5) + rng.normal(0, 2)
    if cond["noise"] == "quiet":
        noise, snr = rng.standard_normal(n), snr + 15
    elif cond["noise"] == "music":
        noise = music(rng, n)
    elif cond["noise"] == "crowd":
        noise = crowd(rng, n, babble_pool)
    else:
        noise = fan(rng, n)
    p_speech = np.mean(speech**2) + 1e-12
    p_noise = np.mean(noise**2) + 1e-12
    noise *= math.sqrt(p_speech / (p_noise * 10 ** (snr / 10)))
    mix = speech + noise
    return (0.5 * mix / (np.abs(mix).max() + 1e-9)).astype(np.float32), round(snr, 1)


# --- Extraction (rule-based stand-in for the BAML extraction LLM) ---


def clean(text):
    return re.sub(r"[^a-z0-9' -]", " ", text.lower()).strip()


def extract(slot, transcription):
    text = TRAILERS.sub("", clean(transcription)).strip()
    if slot == "name":
        value = NAME_CARRIERS.sub("", text).strip()
        value = " ".join(value.split()[:2]).title()
    else:
        value = ARTICLES.sub("", DRINK_CARRIERS.sub("", text)).strip()
        value = LLM_NORMALIZATION.get(value.replace(" ", "").replace("-", ""), value)
    return value or None


# --- Dataset ---


def make_speakers(voices, rng):
    speakers = []
    for i, voice in enumerate(voices):
        locale = voice["Locale"]
        speakers.append(
            {
                "speaker_id": f"tts{i:02d}",
                "voice": voice["ShortName"],
                "accent": locale,
                "native": locale.split("-")[1] in ("US", "CA", "GB", "AU", "NZ", "IE"),
                "noise": CONDITIONS["noise"][i % 4],
                "snr": rng.choice(CONDITIONS["snr"]),
                "distance": rng.choice(CONDITIONS["distance"]),
            }
        )
    return speakers


def condition(sp):
    return ",".join(
        [
            "native accent" if sp["native"] else "non-native accent",
            sp["noise"],
            f"{sp['distance']}m",
            f"SNR {sp['snr']} dB",
        ]
    )


def pick(rng, slot, names, out_ratio=0.25):
    out = rng.random() < out_ratio
    if slot == "name":
        intended = rng.choice(OUT_OF_LIST_NAMES if out else names)
        return intended, intended, out
    intended = rng.choice(OUT_OF_LIST_DRINKS if out else DRINKS)
    spoken = intended
    if intended in SPOKEN_ALIASES and rng.random() < 0.6:
        spoken = SPOKEN_ALIASES[intended]
    return intended, spoken, out


async def synthesize(jobs, cache_dir, concurrency=8):
    import edge_tts

    sem = asyncio.Semaphore(concurrency)

    async def one(job):
        path = os.path.join(cache_dir, job["file"])
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return
        async with sem:
            for _ in range(4):
                try:
                    await edge_tts.Communicate(
                        job["text"], job["voice"], rate=job["rate"]
                    ).save(path)
                    return
                except Exception as e:  # noqa: BLE001 - network hiccups, retry
                    print("tts retry", job["file"], e)
                    await asyncio.sleep(2)

    await asyncio.gather(*(one(j) for j in jobs))


def transcribe(model, audio, slot, names):
    hotwords = ", ".join(names) if slot == "name" else HOT_DRINKS
    question = (
        "What is your name?" if slot == "name" else "What is your favorite drink?"
    )
    segments, _ = model.transcribe(
        audio,
        language="en",
        hotwords=hotwords,
        initial_prompt=f"The question '{question}' was asked",
        condition_on_previous_text=False,
        vad_filter=True,
        word_timestamps=True,
        hallucination_silence_threshold=0.3,
    )
    segments = list(segments)
    words, confs = [], []
    for s in segments:
        for w in s.words or []:
            words.append(w.word.strip())
            confs.append(round(float(w.probability), 4))
    meta = {}
    if segments:
        meta = {
            "avg_logprob": round(
                sum(s.avg_logprob for s in segments) / len(segments), 4
            ),
            "no_speech_prob": round(
                sum(s.no_speech_prob for s in segments) / len(segments), 4
            ),
        }
    return " ".join(words), words, confs, meta


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--out", default=os.path.join(os.path.dirname(__file__), "data", "tts")
    )
    parser.add_argument(
        "--trials", type=int, default=6, help="name+drink dialogs per speaker"
    )
    parser.add_argument("--model", default="distil-large-v3")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=1384)
    args = parser.parse_args()

    import edge_tts

    add_cuda_dlls()
    from faster_whisper import WhisperModel

    rng = random.Random(args.seed)
    nrng = np.random.default_rng(args.seed)
    with open(os.path.join(REPO, "frida_constants", "data", "names.json")) as f:
        names = json.load(f)["names"]

    voices = asyncio.run(edge_tts.list_voices())
    voices = sorted(
        (
            v
            for v in voices
            if v["Locale"].startswith("en-")
            and "Multilingual" not in v["ShortName"]
            and "Expressive" not in v["ShortName"]
        ),
        key=lambda v: v["ShortName"],
    )
    speakers = make_speakers(voices, rng)
    cache = os.path.join(args.out, "audio")
    os.makedirs(cache, exist_ok=True)
    os.makedirs(os.path.join(args.out, "logs"), exist_ok=True)

    # Babble for the crowd condition: other voices saying unrelated sentences
    babble_text = [
        "Did you see the game last night?",
        "I think the kitchen is over there.",
        "We should leave before it gets dark.",
        "Can you pass me the remote, please?",
        "The meeting was moved to Thursday.",
        "She bought a new bike last week.",
    ]
    babble_jobs = [
        {
            "file": f"babble_{i}.mp3",
            "text": t,
            "voice": voices[(i * 7) % len(voices)]["ShortName"],
            "rate": "+0%",
        }
        for i, t in enumerate(babble_text * 2)
    ]
    asyncio.run(synthesize(babble_jobs, cache))
    babble_pool = [load_audio(os.path.join(cache, j["file"])) for j in babble_jobs]

    device = args.device
    if device == "auto":
        import ctranslate2

        device = "cuda" if ctranslate2.get_cuda_device_count() else "cpu"
    model = WhisperModel(
        args.model,
        device=device,
        compute_type="float16" if device == "cuda" else "int8",
    )
    print(f"{len(speakers)} speakers, model {args.model} on {device}")

    # Slots: one ask_and_confirm call each
    slots = []
    for sp in speakers:
        for trial in range(args.trials):
            for slot in ("name", "drink"):
                intended, spoken, out = pick(rng, slot, names)
                slots.append(
                    {
                        "sp": sp,
                        "trial": trial,
                        "slot": slot,
                        "intended": intended,
                        "spoken": spoken,
                        "out": out,
                        "done": False,
                    }
                )

    records = {sp["speaker_id"]: [] for sp in speakers}
    for attempt_idx in range(1, 4):
        pending = [s for s in slots if not s["done"]]
        jobs = []
        for s in pending:
            text = rng.choice(TEMPLATES[s["slot"]]).format(s["spoken"])
            rate = rng.choice(["-10%", "+0%", "+0%", "+15%"])
            key = hashlib.sha1(
                f"{s['sp']['voice']}|{text}|{rate}".encode()
            ).hexdigest()[:12]
            s["job"] = {
                "file": f"{s['sp']['speaker_id']}_{key}.mp3",  # cache key: voice, text, rate
                "text": text,
                "voice": s["sp"]["voice"],
                "rate": rate,
            }
            jobs.append(s["job"])
        print(f"attempt {attempt_idx}: synthesizing {len(jobs)} utterances")
        asyncio.run(synthesize(jobs, cache))

        for s in pending:
            sp = s["sp"]
            audio, snr = degrade(
                nrng, load_audio(os.path.join(cache, s["job"]["file"])), sp, babble_pool
            )
            transcription, words, confs, meta = transcribe(
                model, audio, s["slot"], names
            )
            extracted = extract(s["slot"], transcription) if transcription else None
            correct = bool(extracted) and same_value(
                extracted, s["intended"], s["slot"]
            )
            features = (
                cp.compute_features(
                    extracted, transcription, words, confs, stt_meta=meta
                )
                if extracted
                else None
            )
            question = (
                "What is your name?"
                if s["slot"] == "name"
                else "What is your favorite drink?"
            )
            records[sp["speaker_id"]].append(
                {
                    "timestamp": f"tts-{sp['speaker_id']}-{s['trial']}-{s['slot']}-{attempt_idx}",
                    "speaker_id": sp["speaker_id"],
                    "condition": condition(sp),
                    "data_source": "tts",
                    "voice": sp["voice"],
                    "spoken_text": s["job"]["text"],
                    "speaking_rate": s["job"]["rate"],
                    "snr_db": snr,
                    "slot": s["slot"],
                    "intended": s["intended"],
                    "out_of_list": s["out"],
                    "trial": s["trial"],
                    "task": "hric",
                    "policy": "legacy",
                    "question": question,
                    "query": "name" if s["slot"] == "name" else "LLM_drink",
                    "transcription": transcription,
                    "extracted_value": extracted,
                    "words": words,
                    "confidences": confs,
                    "stt_meta": meta,
                    "options": None,
                    "features": features,
                    "decision": cp.CONFIRM if extracted else "extract_failed",
                    "p_correct": None,
                    "confirm_answer": ("yes" if correct else "no")
                    if extracted
                    else None,
                    "attempt_idx": attempt_idx,
                }
            )
            s["done"] = correct

    total = 0
    for sp in speakers:
        path = os.path.join(args.out, "logs", f"hric_{sp['speaker_id']}.jsonl")
        with open(path, "w", encoding="utf-8") as f:
            for rec in sorted(
                records[sp["speaker_id"]],
                key=lambda r: (r["trial"], r["slot"], r["attempt_idx"]),
            ):
                f.write(json.dumps(rec) + "\n")
                total += 1
    n_ok = sum(r["confirm_answer"] == "yes" for rs in records.values() for r in rs)
    print(
        f"Wrote {total} attempts ({n_ok} correct) from {len(speakers)} speakers to {args.out}/logs"
    )


if __name__ == "__main__":
    sys.exit(main())
