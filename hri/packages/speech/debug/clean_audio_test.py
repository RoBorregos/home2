import numpy as np
import soundfile as sf
import os


def analyze_simple(noisy_path, clean_path):
    if not os.path.exists(noisy_path) or not os.path.exists(clean_path):
        print("Error: No se encuentran los archivos .wav")
        return

    # Cargar audios
    data_noisy, _ = sf.read(noisy_path)
    data_clean, _ = sf.read(clean_path)

    # Calcular RMS (Energía media)
    rms_noisy = np.sqrt(np.mean(data_noisy**2))
    rms_clean = np.sqrt(np.mean(data_clean**2))

    # Calcular Decibelios (evitando log(0))
    db_noisy = 20 * np.log10(rms_noisy + 1e-9)
    db_clean = 20 * np.log10(rms_clean + 1e-9)

    reduction = db_noisy - db_clean

    print("\n" + "=" * 40)
    print("📊 REPORTE DE REDUCCIÓN DE RUIDO")
    print("=" * 40)
    print(f"Volumen Original: {db_noisy:.2f} dB")
    print(f"Volumen Filtrado: {db_clean:.2f} dB")
    print("-" * 40)
    print(f"⚡️ REDUCCIÓN TOTAL: {reduction:.2f} dB")
    print("-" * 40)

    # Dibujar barras en CLI
    def get_bar(db):
        # Normalizamos de -60dB (silencio) a 0dB (máximo)
        length = int(max(0, (db + 60) * 0.8))
        return "█" * length

    print(f"RUIDOSO: {get_bar(db_noisy)}")
    print(f"LIMPIO:  {get_bar(db_clean)}")

    if reduction > 5:
        print("\nConclusión: Se eliminó ruido con éxito.")
    else:
        print("\nConclusión: Diferencia mínima. Revisa si grabaste en silencio.")


if __name__ == "__main__":
    analyze_simple("noisy_raw.wav", "enhanced_output.wav")
