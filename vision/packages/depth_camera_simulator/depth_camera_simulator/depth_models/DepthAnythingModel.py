import numpy as np
from depth_camera_simulator.depth_models.DepthModel import DepthModel

import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
from PIL import Image


class DepthAnythingModel(DepthModel):
    def __init__(self):
        super().__init__()
        self.model = None
        self.processor = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def load_model(
        self, model_path: str = "depth-anything/Depth-Anything-V2-Small-hf"
    ) -> None:
        """
        Load the depth estimation model from the given path or Hugging Face model hub.
        """
        self.processor = AutoImageProcessor.from_pretrained(model_path)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_path).to(
            self.device
        )
        self.model.eval()

    def inference(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Perform inference to extract a depth image from an RGB image.

        Args:
            rgb_image (np.ndarray): Input RGB image.

        Returns:
            np.ndarray: Output depth image.
        """
        if self.model is None or self.processor is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        # Convert numpy array to PIL Image
        pil_image = Image.fromarray(rgb_image.astype(np.uint8))

        # Preprocess
        inputs = self.processor(images=pil_image, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Inference
        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth

        # Resize to original image size and convert to numpy
        prediction = (
            torch.nn.functional.interpolate(
                predicted_depth.unsqueeze(1),
                size=pil_image.size[::-1],  # (height, width)
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )

        # Normalize depth to 0-1
        prediction = (prediction - prediction.min()) / (
            prediction.max() - prediction.min() + 1e-8
        )
        return prediction
