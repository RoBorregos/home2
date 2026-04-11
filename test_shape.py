import numpy as np

pos = np.load(
    "hri/packages/speech/training/oww/.data/models/frida/positive_features_test.npy"
)
neg = np.load(
    "hri/packages/speech/training/oww/.data/models/frida/negative_features_test.npy"
)
print("Pos shape:", pos.shape)
print("Neg shape:", neg.shape)
