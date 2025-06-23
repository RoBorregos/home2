from faster_whisper_backend import ServeClientFasterWhisper

import grpc
from concurrent import futures
import speech_pb2
import speech_pb2_grpc
import argparse

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
        self.add_frames(first_chunk.audio_data)
        prev_len = 0
        
        for chunk in request_iterator:
            client.add_frames(chunk.audio_data)
            if len(client.segments != prev_len):
                prev_len = len(client.segments)
                text = "".join(client.segments)
                yield speech_pb2.TextResponse(text=text)

def serve(port, model, log_transcriptions):
    # Create the gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    speech_pb2_grpc.add_SpeechServiceServicer_to_server(
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

if __name__ == "__main__":
    
    client = ServeClientFasterWhisper(initial_prompt="Roborregos",
                                      send_last_n_segments=10,
                                      clip_audio=False,
                                      # language=options["language"],
                                      # task=options["task"],
                                      # model=options["model"],
                                      same_output_threshold=10)
    print("Client created")
    # client.add_frames(frame_np)
