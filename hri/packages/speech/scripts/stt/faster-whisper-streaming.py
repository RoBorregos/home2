from faster_whisper_backend import ServeClientFasterWhisper

import grpc
from concurrent import futures
import speech_pb2
import speech_pb2_grpc
import argparse
import numpy as np

class WhisperServicer(speech_pb2_grpc.SpeechStreamServicer):
    def __init__(self, model, log_transcriptions=False):
        self.log_transcriptions = log_transcriptions
        self.model = model

    def Transcribe(self, request_iterator, context):
        
        first_chunk = next(request_iterator)
        self.client = ServeClientFasterWhisper(initial_prompt=first_chunk.hotwords,send_last_n_segments=10,clip_audio=False,
                                      # language=options["language"],
                                      # task=options["task"],
                                      # model=options["model"],
                                      same_output_threshold=10)
        
        first_audio = WhisperServicer.bytes_to_float_array(first_chunk.audio_data)
        self.client.add_frames(first_audio)
        prev_len = 0
        
        for chunk in request_iterator:
            frame_np = WhisperServicer.bytes_to_float_array(chunk.audio_data)
            self.client.add_frames(frame_np)
            if len(self.client.segments) != prev_len:
                prev_len = len(self.client.segments)
                print("client segments:", self.client.segments)
                text = "".join([segment['text'] for segment in self.client.segments])
                print("client text:", text)
                yield speech_pb2.TextResponse(text=text)

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
    speech_pb2_grpc.add_SpeechStreamServicer_to_server(
        WhisperServicer(model, log_transcriptions), server
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

