import pyaudio
import numpy as np
import librosa
from scipy.spatial.distance import cosine

# --- Configuración ---
CHUNK = 2048  # Aumentamos un poco para mejor análisis de frecuencia
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
SILENCE_LIMIT = 3  # Segundos para resetear el Lock
ENERGY_THRESHOLD = 700  # Ajusta según tu ruido de fondo
CORR_THRESHOLD = 0.55  # Umbral de periodicidad (voz humana)
SIMILARITY_THRESHOLD = 0.80  # Qué tanto debe parecerse a tu inicio de frase


class AdaptiveSpeakerVAD:
    def __init__(self):
        self.target_mfcc = None
        self.is_locked = False

    def get_features(self, audio_data):
        # Extraer MFCCs (Huella digital del timbre)
        mfccs = librosa.feature.mfcc(y=audio_data, sr=RATE, n_mfcc=13)
        return np.mean(mfccs, axis=1)

    def is_periodic(self, audio_data):
        # Autocorrelación para detectar si es ruido o voz
        corr = np.correlate(audio_data, audio_data, mode="full")
        corr = corr[len(corr) // 2 :]
        low_lag, high_lag = int(RATE / 255), int(RATE / 85)
        if len(corr) > high_lag:
            peak = np.max(corr[low_lag:high_lag])
            return (peak / corr[0]) > CORR_THRESHOLD
        return False

    def process_frame(self, data):
        audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32)
        rms = np.sqrt(np.mean(audio_data**2))

        # 1. Filtro básico de energía
        if rms < ENERGY_THRESHOLD:
            return False, "Silencio"

        # 2. Verificar si es voz humana (Pitch)
        if not self.is_periodic(audio_data):
            return False, "Ruido no vocal"

        # 3. Lógica de Identidad (Timbre)
        current_mfcc = self.get_features(audio_data)

        if not self.is_locked:
            # Bloqueamos el timbre de la primera persona que habla fuerte
            self.target_mfcc = current_mfcc
            self.is_locked = True
            return True, "Lock-on: Hablante Principal"
        else:
            # Comparamos con el dueño de la frase actual
            similarity = 1 - cosine(current_mfcc, self.target_mfcc)
            if similarity > SIMILARITY_THRESHOLD:
                return True, f"Siguiendo Voz (Sim: {similarity:.2f})"
            else:
                return False, f"Voz Ajena (Sim: {similarity:.2f})"

    def reset(self):
        self.target_mfcc = None
        self.is_locked = False


# --- Ejecución ---
vad = AdaptiveSpeakerVAD()
p = pyaudio.PyAudio()
stream = p.open(
    format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK
)

print("* Sistema Adaptativo Listo. Habla de frente para iniciar...")

silence_time = 0
try:
    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        speech_detected, status = vad.process_frame(data)

        if speech_detected:
            silence_time = 0
            print(f"[{status}] - ESCUCHANDO...          ", end="\r")
        else:
            silence_time += CHUNK / RATE
            print(f"[{status}] - PAUSA: {silence_time:.1f}s          ", end="\r")

        # Si hay silencio o voz extraña por mucho tiempo, reseteamos el Lock
        if silence_time > SILENCE_LIMIT:
            if vad.is_locked:
                print("\n\n* Frase terminada. Liberando Lock del hablante...")
                vad.reset()
            silence_time = 0

except KeyboardInterrupt:
    print("\nDetenido.")

stream.stop_stream()
stream.close()
p.terminate()
