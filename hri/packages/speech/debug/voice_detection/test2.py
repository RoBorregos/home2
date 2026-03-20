import pyaudio
import numpy as np

# --- Configuración Refinada ---
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
# Sube esto para ser más estricto con la "claridad" de la voz (0.6 - 0.8)
THRESHOLD = 0.5
# Sube esto para ignorar voces de fondo lejanas o ruido (1000 - 3000)
ENERGY_THRESHOLD = 750
SILENCE_LIMIT = 1.5
VOWEL_FREQ_RANGE = (85, 255)  # Rango más apretado para voz humana estándar


def is_speech(data, rate):
    audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32)

    # 1. Filtro de Energía Absoluta (Primer filtro de distancia)
    # Calculamos la raíz cuadrada de la media de los cuadrados (RMS)
    rms = np.sqrt(np.mean(audio_data**2))
    if rms < ENERGY_THRESHOLD:
        return False

    # 2. Autocorrelación
    corr = np.correlate(audio_data, audio_data, mode="full")
    corr = corr[len(corr) // 2 :]

    low_lag = int(rate / VOWEL_FREQ_RANGE[1])
    high_lag = int(rate / VOWEL_FREQ_RANGE[0])

    if len(corr) > high_lag:
        peak_val = np.max(corr[low_lag:high_lag])
        zero_lag_val = corr[0]

        norm_corr = peak_val / zero_lag_val

        # Debug: Descomenta la siguiente línea para ver tus valores en tiempo real
        # print(f"RMS: {int(rms)} | Corr: {norm_corr:.2f}", end="\r")

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
