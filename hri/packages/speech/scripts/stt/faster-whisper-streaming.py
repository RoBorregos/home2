import argparse
import os
from concurrent import futures

import grpc
import numpy as np
import speech_pb2
import speech_pb2_grpc
from faster_whisper_backend import ServeClientFasterWhisper
from transcriber_faster_whisper import WhisperModel


class WhisperServicer(speech_pb2_grpc.SpeechStreamServicer):
    def __init__(self, model, transcriber=None, log_transcriptions=False):
        self.log_transcriptions = log_transcriptions
        self.model = model
        self.transcriber = transcriber

    def Transcribe(self, request_iterator, context):
        client = None
        try:
            print("Starting transcription...")
            first_chunk = next(request_iterator)

            client = ServeClientFasterWhisper(
<<<<<<< HEAD
                initial_prompt=first_chunk.hotwords,
=======
                hotwords=first_chunk.hotwords,
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
                send_last_n_segments=10,
                clip_audio=False,
                model=self.model,
                language="en",
                task="translate",
                same_output_threshold=10,
                transcriber=self.transcriber,
            )
            print("Hotwords set for transcription:", first_chunk.hotwords)

            first_audio = WhisperServicer.bytes_to_float_array(first_chunk.audio_data)
            client.add_frames(first_audio)

            prev_text = ""

            for chunk in request_iterator:
                try:
                    if len(chunk.audio_data) < 4:
                        continue

                    frame_np = self.bytes_to_float_array(chunk.audio_data)

                    if len(frame_np) < 10:
                        continue

                    client.add_frames(frame_np)
                    text = "".join([segment["text"] for segment in client.segments])

                    if text != prev_text:
                        prev_text = text
                        print("Transcription updated:", str(text))

<<<<<<< HEAD
                        yield speech_pb2.TextResponse(text=text)
=======
                        word_infos = [
                            speech_pb2.WordInfo(
                                word=w["word"],
                                confidence=w["confidence"],
                                start=w["start"],
                                end=w["end"],
                            )
                            for w in client.word_confidences
                        ]

                        yield speech_pb2.TextResponse(text=text, words=word_infos)
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c

                except Exception as e:
                    print(f"Error processing chunk: {str(e)}")
                    continue
        except Exception as e:
            print("Transcription ended")
            if len(str(e)) > 0:
                print("Transcription failed:", str(e))
        finally:
            print("Cleaning up client resources...")
            if client:
                client.cleanup()

    @staticmethod
    def bytes_to_float_array(audio_bytes):
        """
        Convert audio data from bytes to a NumPy float array.

        It assumes that the audio data is in 16-bit PCM format. The audio data is normalized to
        have values between -1 and 1.

        Args:
            audio_bytes (bytes): Audio data in bytes.

        Returns:
            np.ndarray: A NumPy array containing the audio data as float values normalized between -1 and 1.
        """
        raw_data = np.frombuffer(buffer=audio_bytes, dtype=np.int16)
        return raw_data.astype(np.float32) / 32768.0


def serve(port, model, log_transcriptions):
    # Create the gRPC server

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    try:
        import torch

        device = "cuda" if torch.cuda.is_available() else "cpu"
        major, _ = torch.cuda.get_device_capability(device)
        compute_type = "float16" if major >= 7 else "float32"
    except ImportError:
        device = "cpu"
        compute_type = "int8"

    transcriber = WhisperModel(
        model,
        device=device,
        compute_type=compute_type,
        local_files_only=False,
        download_root="./models",
    )

    warmup_file = "./warmup.wav"
    if os.path.exists(warmup_file):
        try:
            print(f"Warming up model with {warmup_file}...")
            result = transcriber.transcribe(warmup_file)
            # Consume generator to complete the warmup
            for _ in result[0]:
                pass
            print("Model warmup complete")
        except Exception as e:
            print(f"Model warmup failed: {e}")
    else:
        print(f"Warmup file {warmup_file} not found, skipping warmup")

    speech_pb2_grpc.add_SpeechStreamServicer_to_server(
        WhisperServicer(model, transcriber, log_transcriptions), server
    )

    # Bind to a port
    server.add_insecure_port(f"0.0.0.0:{port}")
    print(f"Whisper gRPC server is running on port {port}...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Whisper gRPC server")
    parser.add_argument(
        "--port", type=int, default=50051, help="Port to run the gRPC server on"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="base.en",
        help="Model size to use (base.en, large, or small.en)",
    )
    parser.add_argument(
        "--log_transcriptions",
        action="store_true",
        help="Enable logging of transcriptions and audio files",
    )
    args = parser.parse_args()
    serve(args.port, args.model, args.log_transcriptions)
