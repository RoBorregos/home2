import pyaudio
import numpy as np

# --- Configuration ---
CHUNK = 1024  # Samples per frame
FORMAT = pyaudio.paInt16  # 16-bit resolution
CHANNELS = 1  # Mono
RATE = 44100  # Sample rate
THRESHOLD = 0.4  # Correlation strength (0.0 to 1.0)
SILENCE_LIMIT = 1.5  # Seconds of silence before stopping
VOWEL_FREQ_RANGE = (80, 300)  # Human pitch range in Hz


def is_speech(data, rate):
    # Convert buffer to float array and normalize
    audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32)

    # 1. Normalize the signal
    if np.max(np.abs(audio_data)) < 500:  # Ignore very faint background hum
        return False

    # 2. Compute Autocorrelation
    # We only care about lags corresponding to human pitch (80Hz - 300Hz)
    # Lag = Rate / Frequency
    corr = np.correlate(audio_data, audio_data, mode="full")
    corr = corr[len(corr) // 2 :]  # Keep positive lags

    # Calculate indices for the human pitch range
    low_lag = int(rate / VOWEL_FREQ_RANGE[1])
    high_lag = int(rate / VOWEL_FREQ_RANGE[0])

    # Find the peak correlation in the pitch range
    if len(corr) > high_lag:
        peak_val = np.max(corr[low_lag:high_lag])
        zero_lag_val = corr[0]  # The energy of the signal

        # Normalized Correlation Coefficient
        norm_corr = peak_val / zero_lag_val
        return norm_corr > THRESHOLD
    return False


# --- Main Loop ---
p = pyaudio.PyAudio()
stream = p.open(
    format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK
)

print("* Listening... Speak now.")

silence_counter = 0
is_recording = True

try:
    while is_recording:
        data = stream.read(CHUNK, exception_on_overflow=False)

        if is_speech(data, RATE):
            print("Status: [SPEECH DETECTED]", end="\r")
            silence_counter = 0
        else:
            silence_counter += CHUNK / RATE
            print(f"Status: [SILENCE] {silence_counter:.1f}s", end="\r")

        if silence_counter > SILENCE_LIMIT:
            print("\n\n* Main speaker ended talking. Stopping transcription...")
            is_recording = False

except KeyboardInterrupt:
    pass

stream.stop_stream()
stream.close()
p.terminate()
