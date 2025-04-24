import argparse
import os
import sys
from concurrent import futures
from datetime import datetime

import grpc
import speech_pb2
import speech_pb2_grpc
from faster_whisper import WhisperModel

# Add the directory containing the protos to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "speech"))

import shutil

from wav_utils import WavUtils


class WhisperServicer(speech_pb2_grpc.SpeechServiceServicer):
    def __init__(self, model_size, log_transcriptions=False):
        self.model_size = model_size
        self.audio_model = self.load_model()
        self.log_transcriptions = log_transcriptions
        self.log_dir = os.path.join(os.path.dirname(__file__), "logs")
        if self.log_transcriptions:
            os.makedirs(self.log_dir, exist_ok=True)

    def load_model(self):
        model_directory = os.path.join(os.path.dirname(__file__), "models")
        device = "cpu"
        try:
            import torch

            if torch.cuda.is_available():
                device = "cuda"
        except Exception:
            device = "cuda" if torch.cuda.is_available() else "cpu"

        return WhisperModel(
            self.model_size,
            download_root=model_directory,
            device=device,
            compute_type="float32",
        )

    def Transcribe(self, request, context):
        print("Received audio data, transcribing...")

        # Generate a temporary WAV file from received audio data
        temp_file = WavUtils.generate_temp_wav(1, 2, 16000, request.audio_data)

        # Get hotwords from request
        current_hotwords = request.hotwords if request.hotwords else ""

        # Perform transcription
        result = self.audio_model.transcribe(
            temp_file,
            language="en",
            hotwords=current_hotwords,
            condition_on_previous_text=True,
        )

        # Access the generator to collect text segments
        segments = result[0]  # The generator
        transcription = "".join(
            [segment.text for segment in segments]
        )  # Collect all text

        # Log transcription if enabled
        if self.log_transcriptions:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            audio_filename = f"audio_{timestamp}.wav"
            audio_path = os.path.join(self.log_dir, audio_filename)

            shutil.copy(temp_file, audio_path)
            log_file = os.path.join(self.log_dir, "transcriptions.log")
            with open(log_file, "a") as f:
                f.write(
                    f"{timestamp} | Audio: {audio_filename} | Transcription: {transcription}\n"
                )

        WavUtils.discard_wav(temp_file)

        print(f"Transcription: {transcription}")

        # Return the transcribed text
        return speech_pb2.TextResponse(text=transcription)


def serve(port, model_size, log_transcriptions):
    # Create the gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    speech_pb2_grpc.add_SpeechServiceServicer_to_server(
        WhisperServicer(model_size, log_transcriptions), server
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
        "--model_size",
        type=str,
        default="base.en",
        help="Model size to use (base.en, large.en, or small.en)",
    )
    parser.add_argument(
        "--log_transcriptions",
        action="store_true",
        help="Enable logging of transcriptions and audio files",
    )
    args = parser.parse_args()
    serve(args.port, args.model_size, args.log_transcriptions)
