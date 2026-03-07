import argparse
import csv
from concurrent import futures

import grpc
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

import events_pb2
import events_pb2_grpc


class EventsServicer(events_pb2_grpc.EventsServiceServicer):
    TARGET_EVENTS = {
        "door",
        "doorbell",
        "chime",
        "chink, clink",
        "music",
        "tubular bells",
        "cowbell",
        "slap, smack",
    }
    SAMPLE_RATE = 16000
    WINDOW_SECONDS = 1.0
    WINDOW_SAMPLES = int(SAMPLE_RATE * WINDOW_SECONDS)

    CONFIDENCE_THRESHOLD = 0.3

    def __init__(self, model, class_names):
        self.model = model
        self.class_names = class_names

    def Listen(self, request_iterator, context):
        print("Starting event detection...")
        audio_accumulator = np.array([], dtype=np.float32)
        try:
            for chunk in request_iterator:
                try:
                    if len(chunk.audio_data) < 4:
                        continue

                    frame_np = (
                        np.frombuffer(chunk.audio_data, dtype=np.int16).astype(
                            np.float32
                        )
                        / 32768.0
                    )

                    if len(frame_np) < 10:
                        continue

                    audio_accumulator = np.concatenate([audio_accumulator, frame_np])

                    if len(audio_accumulator) < self.WINDOW_SAMPLES:
                        continue

                    scores, embeddings, spectrogram = self.model(audio_accumulator)
                    mean_scores = scores.numpy().mean(axis=0)
                    top_class_idx = mean_scores.argmax()
                    confidence = float(mean_scores[top_class_idx])
                    event_text = self.class_names[top_class_idx]
                    print(f"Detected: {event_text} ({confidence:.3f})")

                    audio_accumulator = np.array([], dtype=np.float32)

                    if confidence < self.CONFIDENCE_THRESHOLD:
                        continue

                    if any(
                        target in event_text.lower() for target in self.TARGET_EVENTS
                    ):
                        print(f"Target event: {event_text} ({confidence:.3f})")
                        yield events_pb2.TextResponse(text=event_text)

                except Exception as e:
                    print(f"Error processing chunk: {str(e)}")
                    continue
        except Exception as e:
            print("Event detection ended")
            if len(str(e)) > 0:
                print("Event detection failed:", str(e))


def serve(port):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    print("Loading YAMNet model...")
    model = hub.load("https://tfhub.dev/google/yamnet/1")
    class_map_path = model.class_map_path().numpy().decode("utf-8")
    with tf.io.gfile.GFile(class_map_path) as f:
        class_names = [row[2] for row in csv.reader(f)]
        class_names = class_names[1:]  # skip header
    print(f"YAMNet model loaded with {len(class_names)} classes.")

    events_pb2_grpc.add_EventsServiceServicer_to_server(
        EventsServicer(model, class_names), server
    )

    server.add_insecure_port(f"0.0.0.0:{port}")
    print(f"Events gRPC server is running on port {port}...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Events gRPC server")
    parser.add_argument(
        "--port", type=int, default=50053, help="Port to run the gRPC server on"
    )
    args = parser.parse_args()
    serve(args.port)
