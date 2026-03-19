import os
from edge_impulse_linux.audio import AudioImpulseRunner

MODEL_FILE = "build/ambient-sound.eim" 
TARGET_LABEL = "knock"

with AudioImpulseRunner(MODEL_FILE) as runner:
    model_info = runner.init()
    labels = model_info['model_parameters']['labels']
    print(f"Etiquetas detectadas por el modelo: {labels}")

    # Este bucle captura el audio del micrófono continuamente
    for res, _ in runner.classifier():
        
        # Extraemos el diccionario con las predicciones
        predictions = res['result']['classification']
        print(predictions)
