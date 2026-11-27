"""Streaming speaker verification used by voice_detection.py."""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

from speech.silero_vad_utils import (
    FRAME_SAMPLES,
    SRC_SPEECH_DIR,
    SileroVAD,
    default_silero_path,
)
from speech.speaker_id_utils import (
    DEFAULT_MODEL_FILE,
    MIXED,
    NOT_TARGET,
    TARGET,
    ProfileStore,
    Segment,
    Segmenter,
    SegmenterConfig,
    SpeakerEmbedder,
    VerifierConfig,
    Window,
    aggregate_segment,
    build_profile,
    decide,
    is_known,
    mixed_parts,
)

PARAMS = {
    "ENABLED": True,
    "AUDIO_TOPIC": "/hri/processedAudioChunk",
    "SILERO_MODEL": "",
    "EMBED_MODEL": "",
    "EMBED_BACKEND": "wespeaker_resnet34_lm",
    "ORT_PROVIDERS": "cuda,cpu",
    "PROFILE_DIR": "",
    "VAD_ON": 0.5,
    "VAD_OFF": 0.4,
    "ENERGY_MARGIN_DB": 3.0,
    "NOISE_FLOOR_TAU_SEC": 3.0,
    "MIN_SPEECH_SEC": 0.25,
    "MIN_SILENCE_SEC": 0.5,
    "PAD_SEC": 0.2,
    "MAX_SEGMENT_SEC": 15.0,
    "TARGET_NAME": "",
    "EMBED_MIN_SEC": 1.0,
    "EMBED_HOP_SEC": 0.5,
    "EMBED_WIN_SEC": 1.5,
    "EMBED_MAX_SEC": 6.0,
    "ACCEPT_TH": 0.38,
    "REJECT_TH": 0.24,
    "MARGIN": 0.10,
    "ADAPT": True,
    "ADAPT_TH": 0.65,
    "ADAPT_MIN_SEC": 2.0,
    "POOL_MAX": 10,
    "ENROLL_MIN_SEC": 3.0,
    "ENROLL_GOOD_SEC": 6.0,
    "ENROLL_OUTLIER_TH": 0.45,
    "ENROLL_TIMEOUT_SEC": 90.0,
    "LOOKBACK_SEC": 30.0,
}


def default_data_dir() -> str:
    if os.path.isdir(SRC_SPEECH_DIR):
        return os.path.join(SRC_SPEECH_DIR, "assets", "downloads", "speaker_id")
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("speech"), "assets", "downloads", "speaker_id"
    )


@dataclass
class Job:
    kind: str
    seg_id: int = 0
    wave: np.ndarray | None = None
    duration: float = 0.0
    snr_db: float = 0.0
    vad_p: float = 0.0
    t_wall_start: float = 0.0
    t_start: float = 0.0
    t_end: float = 0.0
    cut_by_tts: bool = False
    name: str = ""
    seconds: float = 0.0


class SpeakerVerifier:
    def __init__(
        self,
        params: dict,
        on_decision: Callable[[dict], None],
        on_status: Callable[[dict], None],
        logger,
        sample_rate: int = 16000,
    ) -> None:
        self.p = p = {**PARAMS, **params}
        self.on_decision = on_decision
        self.on_status = on_status
        self.log = logger
        self.sr = sample_rate
        data_dir = default_data_dir()
        embed_model = p["EMBED_MODEL"] or os.path.join(data_dir, DEFAULT_MODEL_FILE)
        if not os.path.isfile(embed_model):
            raise FileNotFoundError(
                f"Speaker model not found: {embed_model}. "
                "Run docker/hri/scripts/download-model.sh (option speaker-id)"
            )
        self.seg_cfg = SegmenterConfig(
            sample_rate=self.sr,
            frame_samples=FRAME_SAMPLES,
            vad_on=p["VAD_ON"],
            vad_off=p["VAD_OFF"],
            energy_margin_db=p["ENERGY_MARGIN_DB"],
            noise_floor_tau_sec=p["NOISE_FLOOR_TAU_SEC"],
            min_speech_sec=p["MIN_SPEECH_SEC"],
            min_silence_sec=p["MIN_SILENCE_SEC"],
            pad_sec=p["PAD_SEC"],
            max_segment_sec=p["MAX_SEGMENT_SEC"],
        )
        self.ver_cfg = VerifierConfig(
            accept_th=p["ACCEPT_TH"],
            reject_th=p["REJECT_TH"],
            margin=p["MARGIN"],
            min_sec=p["EMBED_MIN_SEC"],
            adapt_th=p["ADAPT_TH"],
            adapt_min_sec=p["ADAPT_MIN_SEC"],
            pool_max=p["POOL_MAX"],
            enroll_min_sec=p["ENROLL_MIN_SEC"],
            enroll_good_sec=p["ENROLL_GOOD_SEC"],
            enroll_outlier_th=p["ENROLL_OUTLIER_TH"],
        )
        self.backend = p["EMBED_BACKEND"]
        self.target = p["TARGET_NAME"]
        self.vad = SileroVAD(p["SILERO_MODEL"] or default_silero_path(), self.sr)
        providers = [x for x in p["ORT_PROVIDERS"].split(",") if x.strip()]
        self.embedder = SpeakerEmbedder(embed_model, providers, self.sr)
        self.embedder.warmup()
        self.store = ProfileStore(
            p["PROFILE_DIR"] or os.path.join(data_dir, "profiles"), self.backend
        )
        for msg in self.store.load_all():
            self.log.warning(f"Skipped profile {msg}")
        self.segmenter = Segmenter(self.seg_cfg)
        self._pending = np.zeros(0, dtype=np.float32)
        self._seg_wall_start: dict[int, float] = {}
        self._last_partial_t = 0.0
        self._jobs: deque[Job] = deque()
        self._cv = threading.Condition()
        self._lock = threading.Lock()
        self._enrolling = False
        self._enroll_started = 0.0
        self._enroll_waves: list[np.ndarray] = []
        self._lookback: deque[tuple[float, np.ndarray]] = deque()
        self._seg_windows: dict[int, list[Window]] = {}
        self._running = True
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._status("startup", True, "ready")

    def describe(self) -> str:
        return (
            f"embed {self.backend} providers={self.embedder.providers} | profiles: "
            f"{list(self.store.profiles) or 'none'} | target: {self.target or 'any'}"
        )

    def close(self) -> None:
        self._running = False
        with self._cv:
            self._cv.notify_all()

    def push(self, samples: np.ndarray, muted: bool = False) -> None:
        self._pending = np.concatenate(
            [self._pending, samples.astype(np.float32) / 32768.0]
        )
        closed = self.segmenter.mute(muted)
        if closed is not None:
            self._finish_segment(closed)
        while self._pending.shape[0] >= FRAME_SAMPLES:
            frame = self._pending[:FRAME_SAMPLES]
            self._pending = self._pending[FRAME_SAMPLES:]
            prob = self.vad(frame)
            info, started, finished = self.segmenter.push(frame, prob)
            if started is not None:
                self._seg_wall_start[started.seg_id] = time.time()
                self._last_partial_t = 0.0
                self.log.debug(
                    f"[seg {started.seg_id}] speech start "
                    f"(p={prob:.2f} {info.db:.0f}dB floor {info.noise_floor_db:.0f}dB)"
                )
            if finished is not None:
                self._finish_segment(finished)
            elif self.segmenter.current is not None:
                self._maybe_partial(self.segmenter.current)

    def _seg_stats(self, seg: Segment) -> tuple[float, float]:
        probs = np.asarray(seg.probs) if seg.probs else np.zeros(1)
        dbs = np.asarray(seg.dbs) if seg.dbs else np.zeros(1)
        voiced = dbs[probs >= self.seg_cfg.vad_off]
        level = float(np.mean(voiced)) if voiced.size else float(np.mean(dbs))
        return level - self.segmenter.noise_floor_db, float(np.mean(probs))

    def _segment_job(self, kind: str, seg: Segment, max_sec: float) -> Job:
        wave_ = seg.wave()[-int(max_sec * self.sr) :]
        snr, vad_p = self._seg_stats(seg)
        now = time.time()
        final = kind == "final"
        return Job(
            kind,
            seg.seg_id,
            wave_,
            seg.duration(self.sr) if final else wave_.shape[0] / self.sr,
            snr,
            vad_p,
            self._seg_wall_start.get(seg.seg_id, now),
            now - wave_.shape[0] / self.sr,
            now,
            seg.cut_by_tts and final,
        )

    def _maybe_partial(self, seg: Segment) -> None:
        if seg.duration(self.sr) < self.p["EMBED_MIN_SEC"]:
            return
        if self.segmenter.t - self._last_partial_t < self.p["EMBED_HOP_SEC"]:
            return
        self._last_partial_t = self.segmenter.t
        self._enqueue(self._segment_job("partial", seg, self.p["EMBED_WIN_SEC"]))

    def _finish_segment(self, seg: Segment) -> None:
        self._enqueue(self._segment_job("final", seg, self.p["EMBED_MAX_SEC"]))
        self._seg_wall_start.pop(seg.seg_id, None)

    def _enqueue(self, job: Job) -> None:
        with self._cv:
            if job.kind == "partial":
                self._jobs = deque(j for j in self._jobs if j.kind != "partial")
            self._jobs.append(job)
            self._cv.notify()

    def _worker_loop(self) -> None:
        while self._running:
            with self._cv:
                while not self._jobs and self._running:
                    self._cv.wait(timeout=0.5)
                if not self._running:
                    return
                job = self._jobs.popleft()
            try:
                if job.kind in ("partial", "final"):
                    self._process_segment(job)
                elif job.kind == "commit":
                    self._do_commit(job.name, list(self._enroll_waves), "session")
                elif job.kind == "lookback":
                    cutoff = time.time() - job.seconds
                    waves = [w for t, w in self._lookback if t >= cutoff]
                    self._do_commit(job.name, waves, "lookback")
            except Exception as exc:
                self.log.error(f"speaker id error on {job.kind}: {exc!r}")

    def _process_segment(self, job: Job) -> None:
        final = job.kind == "final"
        if final:
            self._remember_segment(job)
        result = {
            "t": round(time.time(), 3),
            "t_start": round(job.t_start, 3),
            "t_end": round(job.t_end, 3),
            "seg_id": job.seg_id,
            "final": final,
            "dur": round(job.duration, 2),
            "snr_db": round(job.snr_db, 1),
            "vad_p": round(job.vad_p, 2),
            "tts_cut": job.cut_by_tts,
            "enrolling": self._enrolling,
            "target": self.target,
        }
        if job.cut_by_tts:
            result["label"] = "TTS_CUT"
        elif job.duration < 0.5 or job.wave is None:
            result["label"] = "TOO_SHORT"
        else:
            emb = self.embedder.embed(job.wave)
            with self._lock:
                dec = decide(
                    emb, self.store.profiles, self.target, job.duration, self.ver_cfg
                )
            result.update(dec.as_dict())
            result["infer_ms"] = round(self.embedder.last_infer_ms, 1)
            result["known"] = is_known(
                dec.best, dec.score, dec.second_score, self.ver_cfg
            )
            result["rejected"] = dec.score <= self.ver_cfg.reject_th
            if not final:
                self._seg_windows.setdefault(job.seg_id, []).append(
                    Window(
                        dec.label,
                        dec.target_score,
                        dec.best,
                        dec.score,
                        job.t_start,
                        job.t_end,
                    )
                )
            else:
                self._aggregate(result, self._seg_windows.get(job.seg_id, []))
                if (
                    self.p["ADAPT"]
                    and not self._enrolling
                    and result["label"] == TARGET
                ):
                    self._maybe_adapt(dec, emb, job.duration)
        if final:
            self._seg_windows.pop(job.seg_id, None)
            result["lat_ms"] = round((time.time() - job.t_end) * 1000.0, 1)
            self._log_final(result)
        else:
            result["since_onset_ms"] = round(
                (time.time() - job.t_wall_start) * 1000.0, 1
            )
        self.on_decision(result)

    def _aggregate(self, result: dict, wins: list[Window]) -> None:
        labels = [w.label for w in wins]
        result["whole_label"] = result["label"]
        result["label"] = aggregate_segment(result["label"], labels)
        result["win_n"] = len(wins)
        result["win_target"] = labels.count(TARGET)
        result["win_not"] = labels.count(NOT_TARGET)
        result["win_max"] = round(max((w.target_score for w in wins), default=0.0), 3)
        if result["label"] == MIXED:
            result["parts"] = mixed_parts(wins, self.ver_cfg)

    def _remember_segment(self, job: Job) -> None:
        if job.wave is None or job.cut_by_tts:
            return
        now = time.time()
        self._lookback.append((now, job.wave))
        while self._lookback and now - self._lookback[0][0] > self.p["LOOKBACK_SEC"]:
            self._lookback.popleft()
        if self._enrolling:
            if now - self._enroll_started > self.p["ENROLL_TIMEOUT_SEC"]:
                self._enrolling = False
                self._enroll_waves = []
                self._status("enroll_timeout", False, "enrollment session expired")
            elif job.duration >= 0.8:
                self._enroll_waves.append(job.wave)
                total = sum(w.shape[0] for w in self._enroll_waves) / self.sr
                self._status(
                    "enroll_progress",
                    True,
                    f"collected {total:.1f}s (+{job.duration:.1f}s)",
                    collected_sec=round(total, 2),
                )

    def _maybe_adapt(self, dec, emb: np.ndarray, duration: float) -> None:
        if dec.label != TARGET or duration < self.ver_cfg.adapt_min_sec:
            return
        name = dec.target or dec.best
        if dec.scores.get(name, 0.0) < self.ver_cfg.adapt_th:
            return
        with self._lock:
            prof = self.store.get(name)
            if prof is None:
                return
            prof.add_to_pool(emb, self.ver_cfg.pool_max)
            self.store.save(prof)

    def _log_final(self, r: dict) -> None:
        enrolling = " [enrolling]" if r["enrolling"] else ""
        if "score" not in r:
            self.log.info(
                f"[seg {r['seg_id']}] {r['dur']:.1f}s {r['label']} "
                f"snr={r['snr_db']:.0f}dB{enrolling}"
            )
            return
        second = f" 2nd={r['second']}:{r['second_score']:.2f}" if r["second"] else ""
        tgt = f" tgt={r['target']}:{r['target_score']:.2f}" if r["target"] else ""
        self.log.info(
            f"[seg {r['seg_id']}] {r['dur']:.1f}s {r['label']:<10} "
            f"best={r['best']}:{r['score']:.2f}{second}{tgt} "
            f"win(T/N/all)={r['win_target']}/{r['win_not']}/{r['win_n']} "
            f"max={r['win_max']:.2f} vad_p={r['vad_p']:.2f} "
            f"snr={r['snr_db']:.0f}dB infer={r['infer_ms']:.0f}ms "
            f"lat={r['lat_ms']:.0f}ms{enrolling}"
        )

    def _do_commit(self, name: str, waves: list[np.ndarray], source: str) -> None:
        res = build_profile(name, waves, self.embedder, self.backend, self.ver_cfg)
        if res.ok and res.profile is not None:
            with self._lock:
                self.store.forget(name)
                path = self.store.save(res.profile)
            self._enrolling = False
            self._enroll_waves = []
            self.log.info(f"ENROLLED ({res.quality}, {source}): {res.message}")
            self._status(
                "enroll_commit",
                True,
                res.message,
                quality=res.quality,
                enrolled_sec=round(res.kept_sec, 2),
                consistency=round(res.consistency, 3),
                path=path,
            )
        else:
            self.log.warning(f"Enrollment of {name} failed: {res.message}")
            self._status(
                "enroll_commit",
                False,
                res.message,
                quality=res.quality,
                enrolled_sec=round(res.kept_sec, 2),
            )

    def command(self, raw: str | dict) -> None:
        if isinstance(raw, dict):
            cmd = raw
        else:
            try:
                raw = raw.strip()
                cmd = json.loads(raw) if raw.startswith("{") else {"op": raw}
            except json.JSONDecodeError as exc:
                self._status("error", False, f"bad JSON: {exc}")
                return
        op = cmd.get("op", "")
        name = str(cmd.get("name", "")).strip()
        if op == "enroll_start":
            self._enrolling = True
            self._enroll_started = time.time()
            self._enroll_waves = []
            self._status(op, True, "collecting speech; send enroll_commit with name")
        elif op == "enroll_commit":
            if not name:
                self._status(op, False, "missing name")
            elif not self._enrolling:
                self._status(op, False, "no enrollment session (send enroll_start)")
            else:
                self._enqueue(Job("commit", name=name))
        elif op == "enroll_abort":
            self._enrolling = False
            self._enroll_waves = []
            self._status(op, True, "enrollment aborted")
        elif op == "enroll_lookback":
            if not name:
                self._status(op, False, "missing name")
            else:
                secs = float(cmd.get("seconds", self.p["LOOKBACK_SEC"]))
                self._enqueue(Job("lookback", name=name, seconds=secs))
        elif op == "set_target":
            if name and self.store.get(name) is None:
                self._status(op, False, f"unknown profile '{name}'")
            else:
                prof = self.store.get(name) if name else None
                self.target = prof.name if prof else ""
                self._status(op, True, f"target = {self.target or 'any enrolled'}")
        elif op == "forget":
            with self._lock:
                ok = self.store.forget(name) if name else False
            if ok and self.target and self.store.get(self.target) is None:
                self.target = ""
            self._status(op, ok, f"forgot {name}" if ok else f"no profile '{name}'")
        elif op == "reload":
            with self._lock:
                skipped = self.store.load_all()
            self._status(op, True, f"reloaded; skipped: {skipped}")
        elif op == "list":
            self._status(op, True, "profiles")
        else:
            self._status("error", False, f"unknown op '{op}'")

    def _status(self, op: str, ok: bool, message: str, **extra) -> None:
        payload = {
            "t": round(time.time(), 3),
            "op": op,
            "ok": ok,
            "message": message,
            "target": self.target,
            "enrolling": self._enrolling,
            "profiles": [p.summary() for p in self.store.profiles.values()],
        }
        payload.update(extra)
        self.on_status(payload)
        if op not in ("startup", "list", "enroll_progress"):
            (self.log.info if ok else self.log.warning)(f"[speaker id {op}] {message}")
