"""Guided speaker-verification session (runs on your laptop, no ROS needed).

Tells the speaker where to stand and which phrase to say, and logs the wall-clock
time of every prompt to guide_labels.csv. Join it with the speaker_verification
log / rosbag to score each phrase (the Orin and laptop clocks must be in sync).

    python3 guided_session.py [start_block]
"""

import csv
import os
import sys
import time

OUT = os.path.abspath("guide_labels.csv")  # written to the current directory
PHRASES = [
    "¿Qué hora es?",
    "Ve a la cocina",
    "Agárrame la botella",
    "Sí",
    "No",
    "Mi nombre es Mario",
    "Quiero un vaso de agua fría",
    "Ya terminé con esta prueba",
]
BLOCKS = [
    ("S1_frente_1m", "DE FRENTE al robot, a ~1 m del micrófono (ReSpeaker)."),
    ("S3_volteado_1.5m", "DE ESPALDAS al robot, a ~1.5 m."),
    ("S2_lejos_3m", "DE FRENTE al robot, a ~3 m."),
    ("S4_lado_1m", "A UN COSTADO del robot (90°, fuera de la cámara), a ~1 m."),
    (
        "S9_silencio",
        "Quédate EN SILENCIO 20 s (no hables tú; si otros hablan, está bien).",
    ),
]
B, G, Y, C, R = "\033[1m", "\033[92m", "\033[93m", "\033[96m", "\033[0m"


def countdown(sec, text):
    for k in range(sec, 0, -1):
        print(f"\r   {text} {k}s ", end="", flush=True)
        time.sleep(1)
    print("\r" + " " * 60 + "\r", end="")


def main():
    start_block = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    new = not os.path.exists(OUT)
    f = open(OUT, "a", newline="")
    w = csv.writer(f)
    if new:
        w.writerow(["unix_time", "block", "idx", "phrase"])
    os.system("clear")
    print(f"{B}Sesión guiada de verificación de hablante{R}")
    print("Mira la otra ventana (monitor) para ver lo que detecta el robot.\n")
    for bi, (name, where) in enumerate(BLOCKS[start_block:], start=start_block):
        print(f"{C}{B}=== Bloque {bi + 1}/{len(BLOCKS)}: {name} ==={R}")
        print(f"{Y}Colócate: {where}{R}")
        input(f"Cuando estés en posición y en silencio, presiona {B}Enter{R}... ")
        countdown(3, "Empezamos en")
        if name == "S9_silencio":
            w.writerow([f"{time.time():.3f}", name, "silence_start", ""])
            countdown(20, "silencio...")
            w.writerow([f"{time.time():.3f}", name, "end", ""])
            f.flush()
            print(f"   {G}Bloque {name} terminado.{R}\n")
            continue
        for i, phrase in enumerate(PHRASES):
            print(f"   {G}{B}DI AHORA:{R} {B}«{phrase}»{R}")
            w.writerow([f"{time.time():.3f}", name, i, phrase])
            f.flush()
            time.sleep(2.5)
            countdown(2, "pausa (silencio)")
        print(f"   {G}Bloque {name} terminado.{R}\n")
        w.writerow([f"{time.time():.3f}", name, "end", ""])
        f.flush()
    print(f"{B}{G}¡Listo! Terminaste todos los bloques. Avísame en el chat.{R}")
    input("Enter para cerrar.")


if __name__ == "__main__":
    main()
