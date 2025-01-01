# python3 debug/speaker.py

from speech.speech_api_utils import SpeechApiUtils

from speech.wav_utils import WavUtils

import os

import sounddevice as sd
from pygame import mixer


def get_devices():
    devices = sd.query_devices()
    num_dev = 0
    for device_info in devices:
        print(
            f"Device [{num_dev}]: [{device_info['name']}], [{device_info['max_input_channels']}] input channels, [{device_info['max_output_channels']}] output channels")
        num_dev = num_dev + 1

    # print("Original order:")
    # print(devices)


def play_audio(file_path):
    mixer.pre_init(frequency=48000, buffer=2048)
    mixer.init()
    mixer.music.load(file_path)
    mixer.music.play()
    while mixer.music.get_busy():
        pass


if __name__ == "__main__":
    get_devices()

    # mp3_path = "play.mp3"
    # WavUtils.play_mp3(mp3_path, device_index=4)
    # play_audio("/workspace/ws/src/speech/scripts/play_temp.wav")

    # wav_path = "recorded_audio.wav"
    # wav_path = "play_temp.wav"
    # WavUtils.play_mp3(mp3_path, device_index=12)
    # WavUtils.play_wav(wav_path, device_index=12)
    # SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
    # SPEAKER_INPUT_CHANNELS = int(
    #     os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
    # SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

    # OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(
    #     SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS)

    # print("OUTPUT_DEVICE_INDEX =", OUTPUT_DEVICE_INDEX)
