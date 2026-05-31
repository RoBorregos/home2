"""InsightFace-based face recognition model — no ROS dependencies."""

import os

import cv2
import numpy as np
from insightface.app import FaceAnalysis

INSIGHTFACE_MODEL = "buffalo_sc"
MATCH_THRESHOLD = 0.35
TRACK_THRESHOLD = 50


def _insightface_providers() -> list:
    cache_dir = os.environ.get("TENSORRT_CACHE_DIR")
    return [
        (
            "TensorrtExecutionProvider",
            {
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": cache_dir,
                "trt_fp16_enable": True,
            },
        ),
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]


def _bbox_area(bbox) -> float:
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


class FaceModel:
    """Wraps InsightFace detection + embedding matching. Stateful (known people list)."""

    def __init__(self, known_faces_path: str, default_name: str = "ale"):
        self.known_faces_path = known_faces_path
        self.default_name = default_name
        self.app: FaceAnalysis | None = None
        self.people_encodings: list[np.ndarray] = []
        self.people_names: list[str] = []

    def load(self) -> None:
        self.app = FaceAnalysis(
            name=INSIGHTFACE_MODEL,
            providers=_insightface_providers(),
        )
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.people_encodings = [np.zeros(512, dtype=np.float32)]
        self.people_names = ["random"]
        self.clear()
        self._load_known_faces()

    def detect(self, frame: np.ndarray) -> list:
        """Return raw InsightFace face objects for frame."""
        return self.app.get(self.apply_clahe(frame))

    def apply_clahe(self, bgr_img: np.ndarray) -> np.ndarray:
        try:
            lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
            l_img, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
            l_img = clahe.apply(l_img)
            return cv2.cvtColor(cv2.merge((l_img, a, b)), cv2.COLOR_LAB2BGR)
        except Exception:
            return bgr_img

    def match(self, query_embedding: np.ndarray) -> str:
        """Return the best-matching known name, or 'Unknown'."""
        if not self.people_encodings:
            return "Unknown"
        distances = self._cosine_distances(self.people_encodings, query_embedding)
        best_idx = int(np.argmin(distances))
        similarity = 1.0 - float(distances[best_idx])
        if similarity >= MATCH_THRESHOLD and best_idx != 0:
            return self.people_names[best_idx]
        return "Unknown"

    def enroll(self, name: str, crop: np.ndarray) -> None:
        """Save face crop to disk and register its embedding."""
        img_name = f"{name}.png"
        save_path = os.path.join(self.known_faces_path, img_name)
        cv2.imwrite(save_path, crop)
        self._process_img(img_name)

    def clear(self) -> None:
        """Remove dynamically enrolled faces; keep default + random."""
        for filename in os.listdir(self.known_faces_path):
            if filename in (".DS_Store", "random.png", f"{self.default_name}.png"):
                continue
            os.remove(os.path.join(self.known_faces_path, filename))

    def _load_known_faces(self) -> None:
        for filename in os.listdir(self.known_faces_path):
            if filename == ".DS_Store":
                continue
            self._process_img(filename)

    def _process_img(self, filename: str) -> None:
        img_path = os.path.join(self.known_faces_path, filename)
        img = cv2.imread(img_path)
        if img is None:
            return
        faces = self.app.get(self.apply_clahe(img))
        if not faces:
            return
        face = max(faces, key=lambda f: _bbox_area(f.bbox))
        self.people_encodings.append(face.embedding.astype(np.float32))
        self.people_names.append(filename[:-4])

    @staticmethod
    def _cosine_distances(known: list[np.ndarray], query: np.ndarray) -> np.ndarray:
        mat = np.stack(known)
        mat = mat / (np.linalg.norm(mat, axis=1, keepdims=True) + 1e-10)
        q = query / (np.linalg.norm(query) + 1e-10)
        return 1.0 - mat @ q
