#!/usr/bin/env python3
"""Speaker verification node (issue #1459).

Runs next to the existing ``voice_detection`` VAD (it does not replace it) and
answers "who is speaking, and is it the target person?":

    audio (int16) -> Silero VAD + adaptive energy gate -> speech segments
                  -> ONNX speaker embedding (WeSpeaker ResNet34)
                  -> cosine vs enrolled profiles -> TARGET / NOT_TARGET / UNKNOWN

Outputs:
    SPEAKER_ID_VAD_TOPIC            Bool   voice activity (any voice)
    SPEAKER_ID_TARGET_ACTIVE_TOPIC  Bool   the target person is speaking now
    SPEAKER_ID_RESULT_TOPIC         String JSON, one per partial/final decision
    SPEAKER_ID_METRICS_TOPIC        String JSON, periodic health/latency metrics
    SPEAKER_ID_STATUS_TOPIC         String JSON, reply to commands (transient local)

Commands (String JSON on SPEAKER_ID_COMMAND_TOPIC), e.g.::

    {"op": "enroll_start"}
    {"op": "enroll_commit", "name": "mario"}
    {"op": "enroll_abort"}
    {"op": "enroll_lookback", "name": "mario", "seconds": 20}
    {"op": "set_target", "name": "mario"}      # "" = any enrolled person
    {"op": "forget", "name": "mario"}
    {"op": "list"} / {"op": "reload"}
"""

from __future__ import annotations

import json
import os
import threading
import time
import wave
from collections import Counter, deque
from dataclasses import dataclass

import numpy as np
import rclpy
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from speech.speaker_id.embedder import DEFAULT_MODEL_FILE, SpeakerEmbedder
from speech.speaker_id.profiles import ProfileStore
from speech.speaker_id.segmenter import Segment, Segmenter, SegmenterConfig
from speech.speaker_id.silero import FRAME_SAMPLES, SileroVAD
from speech.speaker_id.verifier import (
    NOT_TARGET,
    TARGET,
    VerifierConfig,
    aggregate_segment,
    build_profile,
    decide,
)

SRC_SPEECH_DIR = "/workspace/src/hri/packages/speech"


def _default_data_dir() -> str:
    if os.path.isdir(SRC_SPEECH_DIR):
        return os.path.join(SRC_SPEECH_DIR, "assets", "downloads", "speaker_id")
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("speech"), "assets", "downloads", "speaker_id"
    )


def _default_silero() -> str:
    candidates = [os.path.join(SRC_SPEECH_DIR, "assets", "silero_vad.onnx")]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            os.path.join(
                get_package_share_directory("speech"), "assets", "silero_vad.onnx"
            )
        )
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


@dataclass
class Job:
    kind: str  # "partial" | "final" | "commit" | "lookback"
    seg_id: int = 0
    wave: np.ndarray | None = None
    duration: float = 0.0
    snr_db: float = 0.0
    vad_p: float = 0.0
    t_wall_start: float = 0.0
    t_wall_end: float = 0.0
    cut_by_tts: bool = False
    name: str = ""
    seconds: float = 0.0


class SpeakerVerification(Node):
    def __init__(self) -> None:
        super().__init__("speaker_verification")
        p = self._declare_params()

        self.sr = p["SAMPLE_RATE"]
        data_dir = _default_data_dir()
        embed_model = p["EMBED_MODEL"] or os.path.join(data_dir, DEFAULT_MODEL_FILE)
        profile_dir = p["PROFILE_DIR"] or os.path.join(data_dir, "profiles")
        self.dump_dir = p["DUMP_SEGMENTS_DIR"]
        if self.dump_dir:
            os.makedirs(self.dump_dir, exist_ok=True)

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
        self.p = p
        self.backend = p["EMBED_BACKEND"]
        self.target = p["TARGET_NAME"]

        # Models
        silero_path = p["SILERO_MODEL"] or _default_silero()
        self.vad = SileroVAD(silero_path, self.sr)
        if not os.path.isfile(embed_model):
            raise FileNotFoundError(
                f"Speaker model not found: {embed_model}. "
                "Run docker/hri/scripts/download-speaker-models.sh"
            )
        providers = [x for x in p["ORT_PROVIDERS"].split(",") if x.strip()]
        self.embedder = SpeakerEmbedder(embed_model, providers, self.sr)
        self.embedder.warmup()
        self.store = ProfileStore(profile_dir, self.backend)
        for msg in self.store.load_all():
            self.get_logger().warning(f"Skipped profile {msg}")

        # Streaming state (audio thread)
        self.segmenter = Segmenter(self.seg_cfg)
        self._pending = np.zeros(0, dtype=np.float32)
        self._muted_until = 0.0
        self._saying = False
        self._vad_state = False
        self._last_prob = 0.0
        self._seg_wall_start: dict[int, float] = {}
        self._last_partial_t = 0.0
        self._chunks = 0

        # Worker state
        self._jobs: deque[Job] = deque()
        self._cv = threading.Condition()
        self._lock = threading.Lock()
        self._enrolling = False
        self._enroll_started = 0.0
        self._enroll_waves: list[np.ndarray] = []
        self._lookback: deque[tuple[float, np.ndarray]] = deque()
        self._infer_ms: deque[float] = deque(maxlen=200)
        self._latency_ms: deque[float] = deque(maxlen=200)
        self._labels: Counter = Counter()
        self._drops = 0
        self._segments = 0
        self._target_active = False
        self._current_seg_label: dict[int, str] = {}
        self._seg_windows: dict[int, list[tuple[str, float]]] = {}
        self._running = True

        # ROS I/O
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.vad_pub = self.create_publisher(Bool, p["SPEAKER_ID_VAD_TOPIC"], 10)
        self.target_pub = self.create_publisher(
            Bool, p["SPEAKER_ID_TARGET_ACTIVE_TOPIC"], 10
        )
        self.result_pub = self.create_publisher(
            String, p["SPEAKER_ID_RESULT_TOPIC"], 10
        )
        self.metrics_pub = self.create_publisher(
            String, p["SPEAKER_ID_METRICS_TOPIC"], 10
        )
        self.status_pub = self.create_publisher(
            String, p["SPEAKER_ID_STATUS_TOPIC"], latched
        )
        audio_topic = p["INPUT_AUDIO_TOPIC"] or p["RAW_AUDIO_TOPIC"]
        self.audio_topic = audio_topic
        self.create_subscription(AudioData, audio_topic, self._audio_cb, 50)
        self.create_subscription(Bool, p["SAYING_TOPIC"], self._saying_cb, 10)
        self.create_subscription(
            String, p["SPEAKER_ID_COMMAND_TOPIC"], self._command_cb, 10
        )
        self.create_timer(p["METRICS_PERIOD"], self._publish_metrics)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.vad_pub.publish(Bool(data=False))
        self.target_pub.publish(Bool(data=False))
        self._status("startup", True, "ready")
        self.get_logger().info(
            f"SpeakerVerification ready | in: {audio_topic} | silero "
            f"{'v5' if self.vad.is_v5 else 'v4'} | embed {self.backend} "
            f"dim={self.embedder.dim} providers={self.embedder.providers} "
            f"warmup={self.embedder.last_infer_ms:.1f}ms | profiles: "
            f"{list(self.store.profiles) or 'none'} | target: "
            f"{self.target or 'any'} | accept>={self.ver_cfg.accept_th} "
            f"reject<={self.ver_cfg.reject_th} margin={self.ver_cfg.margin}"
        )

    # ------------------------------------------------------------ params
    def _declare_params(self) -> dict:
        defaults = {
            "RAW_AUDIO_TOPIC": "/hri/rawAudioChunk",
            "INPUT_AUDIO_TOPIC": "",
            "SAYING_TOPIC": "/saying",
            "SPEAKER_ID_VAD_TOPIC": "/hri/speaker_id/vad",
            "SPEAKER_ID_TARGET_ACTIVE_TOPIC": "/hri/speaker_id/target_active",
            "SPEAKER_ID_RESULT_TOPIC": "/hri/speaker_id/result",
            "SPEAKER_ID_METRICS_TOPIC": "/hri/speaker_id/metrics",
            "SPEAKER_ID_COMMAND_TOPIC": "/hri/speaker_id/command",
            "SPEAKER_ID_STATUS_TOPIC": "/hri/speaker_id/status",
            "SAMPLE_RATE": 16000,
            "SILERO_MODEL": "",
            "EMBED_MODEL": "",
            "EMBED_BACKEND": "wespeaker_resnet34_lm",
            "ORT_PROVIDERS": "cuda,cpu",
            "PROFILE_DIR": "",
            "DUMP_SEGMENTS_DIR": "",
            "VAD_ON": 0.5,
            "VAD_OFF": 0.4,
            "ENERGY_MARGIN_DB": 3.0,
            "NOISE_FLOOR_TAU_SEC": 3.0,
            "MIN_SPEECH_SEC": 0.25,
            "MIN_SILENCE_SEC": 0.5,
            "PAD_SEC": 0.2,
            "MAX_SEGMENT_SEC": 15.0,
            "TTS_TAIL_SEC": 0.3,
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
            "METRICS_PERIOD": 1.0,
        }
        out = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            out[name] = self.get_parameter(name).value
        return out

    # ------------------------------------------------------- audio thread
    def _saying_cb(self, msg: Bool) -> None:
        self._saying = bool(msg.data)
        if not msg.data:
            self._muted_until = time.monotonic() + self.p["TTS_TAIL_SEC"]

    def _is_muted(self) -> bool:
        return self._saying or time.monotonic() < self._muted_until

    def _audio_cb(self, msg: AudioData) -> None:
        self._chunks += 1
        samples = np.frombuffer(bytes(msg.data), dtype=np.int16)
        audio = samples.astype(np.float32) / 32768.0
        self._pending = np.concatenate([self._pending, audio])

        muted = self._is_muted()
        closed = self.segmenter.mute(muted)
        if closed is not None:
            self._finish_segment(closed)

        while self._pending.shape[0] >= FRAME_SAMPLES:
            frame = self._pending[:FRAME_SAMPLES]
            self._pending = self._pending[FRAME_SAMPLES:]
            prob = self.vad(frame)
            self._last_prob = prob
            info, started, finished = self.segmenter.push(frame, prob)
            if started is not None:
                self._seg_wall_start[started.seg_id] = time.time()
                self._last_partial_t = 0.0
                self._set_vad(True)
                self.get_logger().debug(
                    f"[seg {started.seg_id}] speech start "
                    f"(p={prob:.2f} {info.db:.0f}dB floor {info.noise_floor_db:.0f}dB)"
                )
            if finished is not None:
                self._finish_segment(finished)
            elif self.segmenter.current is not None:
                self._maybe_partial(self.segmenter.current)

    def _set_vad(self, active: bool) -> None:
        if active != self._vad_state:
            self._vad_state = active
            self.vad_pub.publish(Bool(data=active))

    def _seg_stats(self, seg: Segment) -> tuple[float, float]:
        probs = np.asarray(seg.probs) if seg.probs else np.zeros(1)
        dbs = np.asarray(seg.dbs) if seg.dbs else np.zeros(1)
        voiced = dbs[probs >= self.seg_cfg.vad_off]
        level = float(np.mean(voiced)) if voiced.size else float(np.mean(dbs))
        return level - self.segmenter.noise_floor_db, float(np.mean(probs))

    def _maybe_partial(self, seg: Segment) -> None:
        dur = seg.duration(self.sr)
        if dur < self.p["EMBED_MIN_SEC"]:
            return
        if self.segmenter.t - self._last_partial_t < self.p["EMBED_HOP_SEC"]:
            return
        self._last_partial_t = self.segmenter.t
        # sliding window: who is speaking *now* (segments can mix several voices)
        wave_ = seg.wave()[-int(self.p["EMBED_WIN_SEC"] * self.sr) :]
        snr, vad_p = self._seg_stats(seg)
        self._enqueue(
            Job(
                "partial",
                seg.seg_id,
                wave_,
                wave_.shape[0] / self.sr,
                snr,
                vad_p,
                self._seg_wall_start.get(seg.seg_id, time.time()),
            )
        )

    def _finish_segment(self, seg: Segment) -> None:
        self._set_vad(False)
        self._segments += 1
        snr, vad_p = self._seg_stats(seg)
        self._enqueue(
            Job(
                "final",
                seg.seg_id,
                seg.wave()[-int(self.p["EMBED_MAX_SEC"] * self.sr) :],
                seg.duration(self.sr),
                snr,
                vad_p,
                self._seg_wall_start.pop(seg.seg_id, time.time()),
                time.time(),
                seg.cut_by_tts,
            )
        )

    def _enqueue(self, job: Job) -> None:
        with self._cv:
            if job.kind == "partial":
                # keep only the newest partial; finals/commands are never dropped
                before = len(self._jobs)
                self._jobs = deque(j for j in self._jobs if j.kind != "partial")
                self._drops += before - len(self._jobs)
            self._jobs.append(job)
            self._cv.notify()

    # ------------------------------------------------------------ worker
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
            except Exception as exc:  # keep the worker alive
                self.get_logger().error(f"worker error on {job.kind}: {exc!r}")

    def _process_segment(self, job: Job) -> None:
        final = job.kind == "final"
        if final:
            self._remember_segment(job)

        result = {
            "t": round(time.time(), 3),
            "seg_id": job.seg_id,
            "final": final,
            "dur": round(job.duration, 2),
            "snr_db": round(job.snr_db, 1),
            "vad_p": round(job.vad_p, 2),
            "tts_cut": job.cut_by_tts,
            "enrolling": self._enrolling,
            "src": self.audio_topic,
        }
        label = "UNKNOWN"
        if job.cut_by_tts and final:
            label = "TTS_CUT"
            result.update({"label": label, "target": self.target})
        elif job.duration < 0.5 or job.wave is None:
            label = "TOO_SHORT"
            result.update({"label": label, "target": self.target})
        else:
            emb = self.embedder.embed(job.wave)
            infer_ms = self.embedder.last_infer_ms
            self._infer_ms.append(infer_ms)
            with self._lock:
                dec = decide(
                    emb, self.store.profiles, self.target, job.duration, self.ver_cfg
                )
            label = dec.label
            result.update(dec.as_dict())
            result["infer_ms"] = round(infer_ms, 1)
            if not final:
                self._seg_windows.setdefault(job.seg_id, []).append(
                    (label, dec.target_score)
                )
            else:
                wins = self._seg_windows.pop(job.seg_id, [])
                labels = [w[0] for w in wins]
                label = aggregate_segment(dec.label, labels)
                result["whole_label"] = dec.label
                result["label"] = label
                result["win_n"] = len(wins)
                result["win_target"] = labels.count(TARGET)
                result["win_not"] = labels.count(NOT_TARGET)
                result["win_max"] = round(max((w[1] for w in wins), default=0.0), 3)
                if self.p["ADAPT"] and not self._enrolling and label == TARGET:
                    self._maybe_adapt(dec, emb, job.duration)

        now = time.time()
        if final:
            lat = (now - job.t_wall_end) * 1000.0
            self._latency_ms.append(lat)
            result["lat_ms"] = round(lat, 1)
        else:
            result["since_onset_ms"] = round((now - job.t_wall_start) * 1000.0, 1)
        self._labels[label if final else f"partial_{label}"] += 1

        # target_active: True while the latest window is judged TARGET
        # (UNKNOWN windows keep the previous state of this segment)
        if label in (TARGET, NOT_TARGET):
            self._current_seg_label[job.seg_id] = label
        active = (not final) and self._current_seg_label.get(job.seg_id) == TARGET
        if final:
            self._current_seg_label.pop(job.seg_id, None)
            self._seg_windows.pop(job.seg_id, None)
        if active != self._target_active:
            self._target_active = active
            self.target_pub.publish(Bool(data=active))

        self.result_pub.publish(String(data=json.dumps(result)))
        if final:
            self._log_final(result)
            self._dump(job, label, result.get("best", ""), result.get("score", 0.0))

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
        if "score" in r:
            second = (
                f" 2nd={r['second']}:{r['second_score']:.2f}" if r["second"] else ""
            )
            tgt = f" tgt={r['target']}:{r['target_score']:.2f}" if r["target"] else ""
            self.get_logger().info(
                f"[seg {r['seg_id']}] {r['dur']:.1f}s {r['label']:<10} "
                f"best={r['best']}:{r['score']:.2f}{second}{tgt} "
                f"win(T/N/all)={r.get('win_target', 0)}/{r.get('win_not', 0)}/"
                f"{r.get('win_n', 0)} max={r.get('win_max', 0):.2f} "
                f"vad_p={r['vad_p']:.2f} snr={r['snr_db']:.0f}dB "
                f"infer={r['infer_ms']:.0f}ms lat={r['lat_ms']:.0f}ms"
                + (" [enrolling]" if r["enrolling"] else "")
            )
        else:
            self.get_logger().info(
                f"[seg {r['seg_id']}] {r['dur']:.1f}s {r['label']} "
                f"snr={r['snr_db']:.0f}dB" + (" [enrolling]" if r["enrolling"] else "")
            )

    def _dump(self, job: Job, label: str, best: str, score: float) -> None:
        if not self.dump_dir or job.wave is None:
            return
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(job.t_wall_end))
        fname = f"{stamp}_seg{job.seg_id:04d}_{label}_{best or 'none'}_{score:.2f}.wav"
        pcm = (np.clip(job.wave, -1, 1) * 32767).astype(np.int16)
        with wave.open(os.path.join(self.dump_dir, fname), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sr)
            wf.writeframes(pcm.tobytes())

    def _do_commit(self, name: str, waves: list[np.ndarray], source: str) -> None:
        res = build_profile(name, waves, self.embedder, self.backend, self.ver_cfg)
        if res.ok and res.profile is not None:
            with self._lock:
                self.store.forget(name)
                path = self.store.save(res.profile)
            self._enrolling = False
            self._enroll_waves = []
            self.get_logger().info(f"ENROLLED ({res.quality}, {source}): {res.message}")
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
            # keep the session open so the task can collect more speech
            self.get_logger().warning(f"Enrollment of {name} failed: {res.message}")
            self._status(
                "enroll_commit",
                False,
                res.message,
                quality=res.quality,
                enrolled_sec=round(res.kept_sec, 2),
            )

    # ---------------------------------------------------------- commands
    def _command_cb(self, msg: String) -> None:
        try:
            cmd = (
                json.loads(msg.data)
                if msg.data.strip().startswith("{")
                else {"op": msg.data.strip()}
            )
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
        self.status_pub.publish(String(data=json.dumps(payload)))
        if op not in ("startup", "list", "enroll_progress"):
            log = self.get_logger().info if ok else self.get_logger().warning
            log(f"[cmd {op}] {message}")

    # ----------------------------------------------------------- metrics
    def _publish_metrics(self) -> None:
        inf = np.asarray(self._infer_ms) if self._infer_ms else np.zeros(1)
        lat = np.asarray(self._latency_ms) if self._latency_ms else np.zeros(1)
        collected = sum(w.shape[0] for w in self._enroll_waves) / self.sr
        metrics = {
            "t": round(time.time(), 3),
            "audio_chunks": self._chunks,
            "vad_prob": round(self._last_prob, 3),
            "in_speech": self.segmenter.in_speech,
            "muted": self._is_muted(),
            "noise_floor_db": round(self.segmenter.noise_floor_db, 1),
            "infer_ms_p50": round(float(np.percentile(inf, 50)), 1),
            "infer_ms_p95": round(float(np.percentile(inf, 95)), 1),
            "final_lat_ms_p95": round(float(np.percentile(lat, 95)), 1),
            "queue": len(self._jobs),
            "partial_drops": self._drops,
            "segments": self._segments,
            "labels": dict(self._labels),
            "target": self.target,
            "target_active": self._target_active,
            "enrolling": self._enrolling,
            "enroll_collected_sec": round(collected, 2),
            "profiles": list(self.store.profiles),
            "accept_th": self.ver_cfg.accept_th,
            "reject_th": self.ver_cfg.reject_th,
            "margin": self.ver_cfg.margin,
        }
        self._chunks = 0
        self.metrics_pub.publish(String(data=json.dumps(metrics)))

    def destroy_node(self) -> None:
        self._running = False
        with self._cv:
            self._cv.notify_all()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerVerification()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
