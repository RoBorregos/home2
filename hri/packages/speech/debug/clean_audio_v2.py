import sounddevice as sd
import soundfile as sf
import numpy as np
import torch
from df.enhance import enhance, init_df, load_audio, save_audio


def run_v2():
    fs = 48000
    duration = 10

    print("\n[1] GRABANDO... ¡Habla fuerte!")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype="float32")
    sd.wait()

    # --- NORMALIZACIÓN ---
    # Subimos el volumen al máximo posible sin distorsionar
    peak = np.max(np.abs(recording))
    if peak > 0:
        recording = recording / peak
        print(f"Normalizado: El volumen subió de {peak:.4f} a 1.0")
    else:
        print("Error: No se detectó sonido. Revisa tu micro.")
        return

    sf.write("noisy_raw.wav", recording, fs)

    print("\n[2] PROCESANDO CON DEEPFILTERNET...")
    model, df_state, _ = init_df()
    audio, _ = load_audio("noisy_raw.wav", sr=df_state.sr())
    enhanced = enhance(model, df_state, audio)

    save_audio("enhanced_output.wav", enhanced, df_state.sr())

    # --- VERIFICACIÓN FINAL ---
    final_peak = torch.max(torch.abs(enhanced)).item()
    print("\n[RESULTADO]")
    print(f"Pico máximo final: {final_peak:.5f}")

    if final_peak < 0.01:
        print("El filtro sigue borrando todo. ¡Acércate más al micro!")
    else:
        print("¡Ahora sí deberías tener audio limpio!")


if __name__ == "__main__":
    run_v2()
