from abc import ABC, abstractmethod
import numpy as np


class DepthModel(ABC):
    def __init__(self):
        super().__init__()

    @abstractmethod
    def load_model(self, model_path: str) -> None:
        """
        Load the depth estimation model from the given path.
        """
        pass

    @abstractmethod
    def inference(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Perform inference to extract a depth image from an RGB image.

        Args:
            rgb_image (np.ndarray): Input RGB image.

        Returns:
            np.ndarray: Output depth image.
        """
        pass
