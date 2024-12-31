import tempfile
import wave
import os
from pydub import AudioSegment
import soundfile as sf
import sounddevice as sd


class WavUtils:
    @staticmethod
    # Convert incoming data (AudioData) to wav file
    def generate_temp_wav(n_channels, sample_width, sample_rate, data):
        # data = bytes(data)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            with wave.open(temp_file, 'w') as wav_file:
                wav_file.setnchannels(n_channels)
                wav_file.setsampwidth(sample_width)
                wav_file.setframerate(sample_rate)

                wav_file.writeframes(data)
            # Return the temporary file name
            return temp_file.name

    @staticmethod
    def discard_wav(file_path):
        if os.path.exists(file_path) and os.path.isfile(file_path) and file_path.endswith('.wav'):
            os.remove(file_path)

    @staticmethod
    # Return if the audio is over the minumum threshold
    def within_time_frame(file_path, min_time, max_time):
        time = WavUtils.get_file_time(file_path)
        if max_time > time and time > min_time:
            return True
        return False

    @staticmethod
    def get_file_time(file_path):
        with wave.open(file_path, 'r') as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames/float(rate * f.getnchannels())
            return duration

    @staticmethod
    def play_wav(file_path, device_index):
        # playsound(file_path,block=True)
        # return
        data, samplerate = sf.read(file_path)
        sd.default.device = device_index
        sd.play(data, samplerate)
        sd.wait()

    @staticmethod
    def mp3_to_wav(mp3_path, wav_path, sample_rate=44100):
        # Load the MP3 file
        audio = AudioSegment.from_mp3(mp3_path)

        # Export the audio to WAV format
        audio.export(wav_path, format="wav", parameters=[
                     "-ar", str(sample_rate)])

    @staticmethod
    def play_mp3(mp3_path, device_index=11):
        wav_name = os.path.splitext(mp3_path)[0] + "_temp" + ".wav"
        WavUtils.mp3_to_wav(mp3_path, wav_name)
        WavUtils.play_wav(wav_name, device_index)
        WavUtils.discard_wav(wav_name)
