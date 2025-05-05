import grpc
from concurrent import futures
import os
import tts_pb2
import tts_pb2_grpc
import time
import numpy as np
from scipy import signal
import wave
from kokoro import KPipeline
from pygame import mixer
import io


class TTSService(tts_pb2_grpc.TTSServiceServicer):
    def __init__(self):
        # Initialize the TTS pipeline once at service startup
        device = "cpu"
        try:
            import torch

            if torch.cuda.is_available():
                device = "cuda"
        except Exception:
            pass
        print("Using device:", device)
        self.pipeline = KPipeline(lang_code="a", device=device)
        # Original sample rate from kokoro
        self.original_sample_rate = 24000  # Hz
        # Target sample rate for audio playback - match ROS2 node's frequency
        self.target_sample_rate = 48000  # Hz (match the ROS2 node's mixer.pre_init)
        # Make sure the models directory exists
        os.makedirs("models", exist_ok=True)

        # Initialize pygame mixer once
        mixer.pre_init(frequency=self.target_sample_rate, buffer=2048)
        mixer.init()

    def Synthesize(self, request, context):
        print(f"Received request to synthesize text: {request.text}")
        try:
            output_path = "models/" + request.output_path

            # Process voice selection - default to af_heart if not specified
            voice = request.model if request.model else "af_heart"
            if voice.endswith(".onnx"):  # Handle legacy piper model paths
                voice = "af_heart"  # Default to af_heart for kokoro

            print(f"Using kokoro with voice: {voice}")

            # Generate audio chunks using kokoro
            generator = self.pipeline(request.text, voice=voice)

            # Create a list to collect all audio samples
            all_audio = []

            # Stream and play each audio chunk
            for i, (gs, ps, audio) in enumerate(generator):
                print(f"Chunk {i}: {gs}, {ps}")

                # Resample if needed
                if self.original_sample_rate != self.target_sample_rate:
                    resampled_audio = signal.resample(
                        audio,
                        int(
                            len(audio)
                            * self.target_sample_rate
                            / self.original_sample_rate
                        ),
                    )
                    audio_chunk = resampled_audio
                else:
                    audio_chunk = audio

                # Add to complete audio collection
                all_audio.append(audio_chunk)

                # Play the audio chunk with pygame mixer
                self._play_audio_chunk(audio_chunk)

            # Concatenate all audio chunks
            complete_audio = np.concatenate(all_audio)

            # Save the complete audio to a WAV file for caching
            self._save_audio_to_wav(
                complete_audio, output_path, self.target_sample_rate
            )

            return tts_pb2.SynthesizeResponse(success=True)

        except Exception as e:
            print(f"Error during kokoro TTS synthesis: {e}")
            return tts_pb2.SynthesizeResponse(success=False, error_message=str(e))

    def _play_audio_chunk(self, audio_data):
        """Play audio chunk using pygame mixer."""
        # Normalize audio to int16
        audio_int16 = np.int16(audio_data * 32767)

        # Create a WAV file in memory
        buffer = io.BytesIO()
        with wave.open(buffer, "wb") as wf:
            wf.setnchannels(1)  # Mono
            wf.setsampwidth(2)  # 16 bits per sample
            wf.setframerate(self.target_sample_rate)
            wf.writeframes(audio_int16.tobytes())

        # Reset buffer position
        buffer.seek(0)

        # Wait until mixer is available
        while mixer.music.get_busy():
            time.sleep(0.05)

        # Load and play from memory buffer
        mixer.music.load(buffer)
        mixer.music.play()

        # Wait until this chunk finishes playing
        while mixer.music.get_busy():
            time.sleep(0.05)

    def _save_audio_to_wav(self, audio_data, output_path, sample_rate):
        """Save audio data to WAV file for caching."""
        # Ensure the directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        # Normalize audio data to prevent clipping
        audio_data = np.int16(audio_data * 32767)

        # Save as WAV file
        with wave.open(output_path, "wb") as wf:
            wf.setnchannels(1)  # Mono
            wf.setsampwidth(2)  # 16 bits per sample
            wf.setframerate(sample_rate)
            wf.writeframes(audio_data.tobytes())

        print(f"Saved audio to {output_path}")


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    tts_pb2_grpc.add_TTSServiceServicer_to_server(TTSService(), server)
    server.add_insecure_port("[::]:50050")
    print("Starting Kokoro TTS server on port 50050...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
