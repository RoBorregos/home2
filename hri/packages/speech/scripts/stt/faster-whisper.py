import argparse
import os
import sys
from concurrent import futures
from datetime import datetime
from pydub import AudioSegment
from pydub.silence import detect_silence

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
            pass

        print("Using device:", device)

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

        audio_path = os.path.join(self.log_dir, "og.wav")
        shutil.copy(temp_file, audio_path)

        audio = AudioSegment.from_wav(temp_file)

        # Analyze audio to determine appropriate silence threshold
        chunk_size = 100  # milliseconds
        chunks = [audio[i : i + chunk_size] for i in range(0, len(audio), chunk_size)]

        # Get dBFS (decibels relative to full scale) for each chunk
        dbfs_values = [chunk.dBFS for chunk in chunks if chunk.dBFS > float("-inf")]

        if not dbfs_values:
            silence_thresh = -28  # Default for silent audio
        else:
            # Sort by loudness (loudest first)
            dbfs_values.sort(reverse=True)

            # Calculate average of the top 30% chunks (the loudest consistent parts)
            top_index = max(1, int(len(dbfs_values) * 0.3))
            top_chunks = dbfs_values[:top_index]
            avg_loud_dbfs = sum(top_chunks) / len(top_chunks)

            # Set threshold below this average to capture all speech but exclude background
            silence_thresh = avg_loud_dbfs - 8

            # Ensure the threshold stays within reasonable bounds
            silence_thresh = max(-50, min(-25, silence_thresh))

        print(f"Using adaptive silence threshold: {silence_thresh} dB")

        # Discard audio if threshold is less than -35 dB (too quiet/noisy)
        if silence_thresh < -35:
            print("Audio too quiet or noisy, discarding")
            WavUtils.discard_wav(temp_file)
            return speech_pb2.TextResponse(text="")

        # Detect silence with our adaptive threshold - reduced min_silence_len for better segmentation
        silence_segments = detect_silence(
            audio, min_silence_len=300, silence_thresh=silence_thresh
        )

        # Invert silence segments to get non-silence segments
        non_silence_segments = []
        if silence_segments:
            if silence_segments[0][0] != 0:
                non_silence_segments.append((0, silence_segments[0][0]))
            for i in range(len(silence_segments) - 1):
                non_silence_segments.append(
                    (silence_segments[i][1], silence_segments[i + 1][0])
                )
            if silence_segments[-1][1] != len(audio):
                non_silence_segments.append((silence_segments[-1][1], len(audio)))
        else:
            non_silence_segments.append((0, len(audio)))

        # Filter out short segments
        min_segment_length = 250  # milliseconds
        non_silence_segments = [
            (start, end)
            for start, end in non_silence_segments
            if (end - start) >= min_segment_length
        ]

        # If all segments were filtered out, keep the original audio
        if not non_silence_segments:
            non_silence_segments = [(0, len(audio))]

        print(
            f"Found {len(non_silence_segments)} speech segments of at least {min_segment_length}ms"
        )

        # Add margins around each non-silence segment to avoid cutting words
        margin_ms = 150  # milliseconds to add before and after each segment
        expanded_segments = []
        for start, end in non_silence_segments:
            # Add margin but don't go below 0 or beyond audio length
            new_start = max(0, start - margin_ms)
            new_end = min(len(audio), end + margin_ms)
            expanded_segments.append((new_start, new_end))

        # Merge overlapping segments after expansion
        if expanded_segments:
            expanded_segments.sort()
            merged_segments = [expanded_segments[0]]
            for current_start, current_end in expanded_segments[1:]:
                prev_start, prev_end = merged_segments[-1]
                if current_start <= prev_end:  # Overlap detected
                    # Merge segments
                    merged_segments[-1] = (prev_start, max(prev_end, current_end))
                else:
                    # No overlap, add as new segment
                    merged_segments.append((current_start, current_end))

            non_silence_segments = merged_segments

        # Create a new audio segment with only the speech parts and small silent padding
        padding_ms = 300  # Amount of silence to add between segments
        processed_audio = AudioSegment.empty()

        # Add each non-silence segment with padding
        for i, (start, end) in enumerate(non_silence_segments):
            # Add the non-silence segment
            processed_audio += audio[start:end]

            # Add padding silence between segments (but not after the last one)
            if i < len(non_silence_segments) - 1:
                processed_audio += AudioSegment.silent(duration=padding_ms)

        # Export the processed audio
        processed_audio.export(temp_file, format="wav")

        # Get hotwords from request
        current_hotwords = request.hotwords if request.hotwords else ""

        # Perform transcription
        result = self.audio_model.transcribe(
            temp_file,
            hotwords=current_hotwords,
            vad_filter=True,
            multilingual=True,
            vad_parameters={
                "threshold": 0.9,
            },
            initial_prompt="only transcribe loudest speaker",
        )

        # Access the generator to collect text segments
        segments = result[0]  # The generator
        transcription = "".join(
            [segment.text for segment in segments]
        )  # Collect all text

        # Log transcription if enabled
        if self.log_transcriptions:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
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
        help="Model size to use (base.en, large, or small.en)",
    )
    parser.add_argument(
        "--log_transcriptions",
        action="store_true",
        help="Enable logging of transcriptions and audio files",
    )
    args = parser.parse_args()
    serve(args.port, args.model_size, args.log_transcriptions)
