import pyaudio
import wave
import numpy as np

# List available input devices and test audio recording
# To check if the audio was recorded correctly, use 'TestSpeaker.py' or see "ws/src/speech/Readme.md"


def record_audio(output_file, input_device_index, duration=10, channels=1, sample_rate=48000, chunk_size=480, format=pyaudio.paInt16):
    audio = pyaudio.PyAudio()

    # Open the audio stream
    # See available devices with list_audio_devices()
    stream = audio.open(input_device_index=input_device_index,
                        format=format,
                        channels=channels,
                        rate=sample_rate,
                        input=True,
                        frames_per_buffer=chunk_size)

    print("Recording...")

    frames = []

    # Record audio data
    for _ in range(0, int(sample_rate / chunk_size * duration)):
        data = stream.read(chunk_size)
        frames.append(data)

    print("Recording stopped.")

    # Close and terminate the audio stream and PyAudio
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # Write audio data to a WAV file
    with wave.open(output_file, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(audio.get_sample_size(format))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))

    print(f"Audio recorded and saved to {output_file}")


def record_audio_respeaker(output_file, input_device_index, duration=10, channels=6, sample_rate=16000, chunk_size=512, format=pyaudio.paInt16, extract_channel=0):
    audio = pyaudio.PyAudio()

    # Open the audio stream
    # See available devices with list_audio_devices()
    stream = audio.open(input_device_index=input_device_index,
                        format=format,
                        channels=channels,
                        rate=sample_rate,
                        input=True,
                        frames_per_buffer=chunk_size)

    print("Recording...")

    frames = []
    frames_6 = []

    # Record audio data
    for _ in range(0, int(sample_rate / chunk_size * duration)):
        data = stream.read(chunk_size)
        a = np.frombuffer(data, dtype=np.int16)[extract_channel::6]
        frames.append(a.tobytes())
        frames_6.append(data)

    print("Recording stopped.")

    # Close and terminate the audio stream and PyAudio
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # Write audio data to a WAV file
    with wave.open("1_" + output_file, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(audio.get_sample_size(format))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))

    with wave.open("6_" + output_file, 'wb') as wf:
        wf.setnchannels(6)
        wf.setsampwidth(audio.get_sample_size(format))
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames_6))

    print(f"Audio recorded and saved to 1_{output_file} and 6_{output_file}")

# Get available devices


def list_audio_devices():
    p = pyaudio.PyAudio()
    num_devices = p.get_device_count()
    print("Available audio devices:")
    for i in range(num_devices):
        device_info = p.get_device_info_by_index(i)
        print(f"Device {i}: [{device_info['name']}], [{device_info['maxInputChannels']}] input channels, [{device_info['maxOutputChannels']}] output channels")
    p.terminate()


if __name__ == "__main__":
    list_audio_devices()

    # output_file = "../recorded_audio.wav"
    # record_audio(output_file, input_device_index=12, duration=5)
    # record_audio_respeaker(output_file, input_device_index=10, duration=10, channels=6, sample_rate=16000, chunk_size=512, format=pyaudio.paInt16, extract_channel=0)
