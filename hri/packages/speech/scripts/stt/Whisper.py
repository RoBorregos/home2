import grpc
from concurrent import futures
import speech_pb2
import speech_pb2_grpc
import whisper
import torch
import argparse
from WavUtils import WavUtils
import os


class WhisperServicer(speech_pb2_grpc.SpeechServiceServicer):
    def __init__(self):
        self.audio_model = self.load_model()

    def load_model(self):
        model_size = "base.en"
        model_directory = os.path.join(os.path.dirname(__file__), "models")
        whisper._download(whisper._MODELS[model_size], model_directory, False)
        return whisper.load_model(model_size, download_root=model_directory)

    def Transcribe(self, request, context):
        # Generate a temporary WAV file from received audio data
        temp_file = WavUtils.generate_temp_wav(1, 2, 16000, request.audio_data)

        # Perform transcription
        result = self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
        WavUtils.discard_wav(temp_file)

        # Return the transcribed text
        return speech_pb2.TextResponse(text=result["text"].strip())


def serve(port):
    # Create the gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    speech_pb2_grpc.add_SpeechServiceServicer_to_server(WhisperServicer(), server)

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
    args = parser.parse_args()
    serve(args.port)
