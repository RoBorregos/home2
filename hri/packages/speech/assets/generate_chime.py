import wave
import math
import struct


def generate_chime(filename, duration=0.5, frequency=880, sample_rate=44100):
    n_samples = int(sample_rate * duration)
    with wave.open(filename, "w") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)

        for i in range(n_samples):
            # Simple sine wave with decay
            t = float(i) / sample_rate
            value = int(
                32767.0 * math.sin(2 * math.pi * frequency * t) * math.exp(-3 * t)
            )
            data = struct.pack("<h", value)
            wav_file.writeframesraw(data)


if __name__ == "__main__":
    generate_chime("listening_chime.wav")
    print("Generated listening_chime.wav")
