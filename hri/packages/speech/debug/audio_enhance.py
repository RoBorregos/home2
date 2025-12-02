#!/usr/bin/env python3
import os
import sys
import time
import tempfile
import argparse
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from frida_interfaces.msg import AudioData

from speech.audio_processing import write_wav


class DebugListener(Node):
    def __init__(self, topic: str, buffer_seconds: int = 3, sample_rate: int = 16000):
        super().__init__("debug_listen_and_speak")
        self.topic = topic
        self.buffer_seconds = buffer_seconds
        self.sr = sample_rate
        self.sub = self.create_subscription(AudioData, topic, self.cb, 50)
        self.frames = []
        self.lock = threading.Lock()

    def cb(self, msg: AudioData):
        if not msg.data:
            return
        with self.lock:
            self.frames.append(bytes(msg.data))

    def get_buffer(self):
        with self.lock:
            data = b"".join(self.frames)
            self.frames = []
        return data


def bytes_to_float_array(b: bytes) -> np.ndarray:
    if not b:
        return np.zeros(0, dtype=np.float32)
    arr = np.frombuffer(b, dtype=np.int16).astype(np.float32) / 32768.0
    return arr


def speak_text(text: str):
    try:
        from gtts import gTTS
        from pygame import mixer

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
        tts = gTTS(text=text, lang="en")
        tts.save(tmp.name)
        mixer.init()
        mixer.music.load(tmp.name)
        mixer.music.play()
        while mixer.music.get_busy():
            time.sleep(0.1)
        mixer.quit()
        try:
            os.remove(tmp.name)
        except Exception:
            pass
    except Exception as e:
        print("TTS playback failed or not available, printing text instead:", e)
        print(text)


def try_transcribe_with_faster_whisper(
    wav_path: str, model_name: str = "base.en"
) -> str:
    try:
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        scripts_stt = os.path.join(repo_root, "scripts", "stt")
        if scripts_stt not in sys.path:
            sys.path.insert(0, scripts_stt)

        from transcriber_faster_whisper import WhisperModel

        model = WhisperModel(
            model_name, device="cpu", compute_type="int8", local_files_only=False
        )
        segments_gen, info = model.transcribe(wav_path, vad_filter=False)
        text = (
            "".join([s.text for s in segments_gen]) if segments_gen is not None else ""
        )
        return text
    except Exception as e:
        print("Failed to transcribe with faster-whisper:", e)
        return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/processedAudioChunk")
    parser.add_argument(
        "--buffer", type=int, default=3, help="Seconds to buffer before transcribing"
    )
    parser.add_argument("--sr", type=int, default=16000)
    parser.add_argument(
        "--model",
        type=str,
        default="base.en",
        help="faster-whisper model id (optional)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = DebugListener(args.topic, buffer_seconds=args.buffer, sample_rate=args.sr)

    print(
        f"Subscribed to {args.topic}. Buffering {args.buffer}s then attempting STT+TTS."
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            with node.lock:
                total_samples = 0
                for chunk in node.frames:
                    total_samples += len(chunk) // 2
                if total_samples >= args.buffer * args.sr:
                    data = node.get_buffer()
                else:
                    data = None

            if data:
                float_arr = bytes_to_float_array(data)
                ts = int(time.time())
                wav_path = os.path.join(
                    tempfile.gettempdir(), f"debug_capture_{ts}.wav"
                )
                try:
                    write_wav(wav_path, float_arr, args.sr)
                    print("Wrote debug wav:", wav_path)
                except Exception as e:
                    print("Failed to write wav:", e)
                    continue

                text = try_transcribe_with_faster_whisper(
                    wav_path, model_name=args.model
                )
                if not text:
                    print(
                        "No transcription available (ensure faster-whisper model is available)."
                    )
                else:
                    print("Transcription:", text)
                    speak_text(text)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
