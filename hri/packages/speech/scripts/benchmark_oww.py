import numpy as np
import onnxruntime as ort
import os
from tqdm import tqdm


def evaluate_model(model_path, pos_features, neg_features, threshold=0.5):
    print(f"Evaluating {model_path} ...")
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    pos_preds = []
    for x in tqdm(pos_features, desc="Positives", leave=False):
        batch = x[np.newaxis, ...]  # shape (1, 16, 96)
        pred = session.run(None, {input_name: batch.astype(np.float32)})[0]
        pos_preds.append(pred.item())

    neg_preds = []
    for x in tqdm(neg_features, desc="Negatives", leave=False):
        batch = x[np.newaxis, ...]  # shape (1, 16, 96)
        pred = session.run(None, {input_name: batch.astype(np.float32)})[0]
        neg_preds.append(pred.item())

    pos_preds = np.array(pos_preds)
    neg_preds = np.array(neg_preds)

    true_positives = np.sum(pos_preds >= threshold)
    false_negatives = np.sum(pos_preds < threshold)

    true_negatives = np.sum(neg_preds < threshold)
    false_positives = np.sum(neg_preds >= threshold)

    recall = (
        true_positives / (true_positives + false_negatives)
        if (true_positives + false_negatives) > 0
        else 0
    )
    precision = (
        true_positives / (true_positives + false_positives)
        if (true_positives + false_positives) > 0
        else 0
    )
    accuracy = (true_positives + true_negatives) / (len(pos_preds) + len(neg_preds))

    print(f"  Recall (TPR):   {recall:.4f} ({true_positives}/{len(pos_preds)})")
    print(f"  Precision:      {precision:.4f}")
    print(f"  Accuracy:       {accuracy:.4f}")
    print(f"  False Positives:{false_positives}\n")


if __name__ == "__main__":
    pos_features_path = (
        "hri/packages/speech/training/oww/.data/models/frida/positive_features_test.npy"
    )
    neg_features_path = (
        "hri/packages/speech/training/oww/.data/models/frida/negative_features_test.npy"
    )

    if not os.path.exists(pos_features_path) or not os.path.exists(neg_features_path):
        print("Test feature files not found in the .data folder.")
        exit(1)

    pos = np.load(pos_features_path)
    neg = np.load(neg_features_path)

    print(
        f"Loaded features: {pos.shape[0]} positive examples, {neg.shape[0]} negative examples.\n"
    )

    model1 = "hri/packages/speech/assets/oww/stop.onnx"
    model2 = "hri/packages/speech/assets/oww_rt/stop.onnx"

    if os.path.exists(model1):
        evaluate_model(model1, pos, neg)
    else:
        print(f"Model not found: {model1}")

    if os.path.exists(model2):
        evaluate_model(model2, pos, neg)
    else:
        print(f"Model not found: {model2}")
