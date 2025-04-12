#! /usr/bin/env python3
from detectors.ObjectDetector import ObjectDectector
import warnings
from abc import abstractmethod

warnings.filterwarnings("ignore", category=FutureWarning)


class ZeroShotObjectDetector(ObjectDectector):
    @abstractmethod
    def set_classes(self, classes: list[str]):
        """Set the classes for the object detector.

        Args:
            classes (list[str]): List of class names.
        """
        pass
