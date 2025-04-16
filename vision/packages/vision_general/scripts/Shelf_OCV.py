# import sys
import math
import cv2 as cv
import numpy as np

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List


@dataclass
class BBox:
    x1: float = 0
    x2: float = 0
    y1: float = 0
    y2: float = 0


@dataclass
class ShelfDetection:
    bbox_: BBox
    level_: int


class ShelfDetector(ABC):
    def __init__(self, image):
        self.image = image
        self.shelves_ = []

    @abstractmethod
    def detect_shelves(self):
        pass

    def get_shelf_detections(
        self, final_horizontal: List[List[int]], filtered_vertical: List[List[int]]
    ) -> List[ShelfDetection]:
        shelf_detections = []

        if len(final_horizontal) >= 2 and len(filtered_vertical) == 2:
            x_left = filtered_vertical[0][0]
            x_right = filtered_vertical[1][0]

            for i in range(len(final_horizontal) - 1):
                y_top = final_horizontal[i][1]
                y_bottom = final_horizontal[i + 1][1]

                detection = ShelfDetection(
                    level_=i + 1,
                    bbox_=BBox(
                        x1=float(x_left),
                        y1=float(y_top),
                        x2=float(x_right),
                        y2=float(y_bottom),
                    ),
                )
                shelf_detections.append(detection)

        return shelf_detections

    def get_shelves(self):
        return self.shelves_


class OpenCVShelfDetector(ShelfDetector):
    def detect_shelves(self):
        THRESHOLD_MIN_VERTICAL = 20
        THRESHOLD_MAX_Y = 10
        THRESHOLD_MIN_HORIZONTAL = 50
        MIN_X_DISTANCE = 200

        src = self.image
        dst = cv.Canny(src, 45, 302, None, 3)
        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is None:
            self.shelves_ = []
            return [], []

        merged_horizontal_lines = []
        horizontal_line_lengths = []

        for i in range(0, len(linesP)):
            line_segment = linesP[i][0]
            dx = abs(line_segment[2] - line_segment[0])
            dy = abs(line_segment[3] - line_segment[1])
            length = math.sqrt(dx**2 + dy**2)
            horizontal_line_lengths.append(length)

            if dy < 10:
                merged = False
                for j in range(len(merged_horizontal_lines)):
                    ml = merged_horizontal_lines[j]
                    if abs(line_segment[1] - ml[1]) < THRESHOLD_MIN_HORIZONTAL and (
                        abs(line_segment[0] - ml[2]) < THRESHOLD_MIN_HORIZONTAL
                        or abs(line_segment[2] - ml[0]) < 100
                    ):
                        merged_horizontal_lines[j] = [
                            min(ml[0], line_segment[0]),
                            ml[1],
                            max(ml[2], line_segment[2]),
                            ml[3],
                        ]
                        merged = True
                        break
                if not merged:
                    merged_horizontal_lines.append(line_segment)

        avg_horizontal_length = sum(horizontal_line_lengths) / len(
            horizontal_line_lengths
        )
        filtered_horizontal_lines = [
            ml
            for ml in merged_horizontal_lines
            if math.sqrt((ml[2] - ml[0]) ** 2 + (ml[3] - ml[1]) ** 2)
            > avg_horizontal_length + 0.675 * np.std(horizontal_line_lengths)
        ]

        final_horizontal_lines = []
        for i in range(len(filtered_horizontal_lines)):
            keep = True
            for j in range(len(final_horizontal_lines)):
                if (
                    abs(filtered_horizontal_lines[i][1] - final_horizontal_lines[j][1])
                    < 80
                ):
                    keep = False
                    break
            if keep:
                final_horizontal_lines.append(filtered_horizontal_lines[i])

        final_horizontal_lines = sorted(
            final_horizontal_lines, key=lambda line: line[1]
        )

        merged_vertical_lines = []
        for i in range(0, len(linesP)):
            line_segment = linesP[i][0]
            dx = abs(line_segment[2] - line_segment[0])
            dy = abs(line_segment[3] - line_segment[1])

            if dx < 50:
                merged = False
                for j in range(len(merged_vertical_lines)):
                    ml = merged_vertical_lines[j]
                    if abs(line_segment[0] - ml[0]) < THRESHOLD_MIN_VERTICAL and (
                        abs(line_segment[1] - ml[3]) < THRESHOLD_MIN_VERTICAL
                        or abs(line_segment[3] - ml[1]) < THRESHOLD_MAX_Y
                    ):
                        merged_vertical_lines[j] = [
                            ml[0],
                            min(ml[1], line_segment[1]),
                            ml[2],
                            max(ml[3], line_segment[3]),
                        ]
                        merged = True
                        break
                if not merged:
                    merged_vertical_lines.append(line_segment)

        sorted_vertical_lines = sorted(
            merged_vertical_lines, key=lambda ml: abs(ml[3] - ml[1]), reverse=True
        )

        filtered_vertical_lines = []
        for i in range(len(sorted_vertical_lines)):
            for j in range(i + 1, len(sorted_vertical_lines)):
                x1 = sorted_vertical_lines[i][0]
                x2 = sorted_vertical_lines[j][0]
                if abs(x1 - x2) >= MIN_X_DISTANCE:
                    filtered_vertical_lines = [
                        sorted_vertical_lines[i],
                        sorted_vertical_lines[j],
                    ]
                    break
            if len(filtered_vertical_lines) == 2:
                break

        self.shelves_ = self.get_shelf_detections(
            final_horizontal_lines, filtered_vertical_lines
        )

        return final_horizontal_lines, filtered_vertical_lines


# if __name__ == "__main__":
#     try:
#         image_path = sys.argv[1] if len(sys.argv) > 1 else "Shelf_Lab.jpg"
#         shelves = detect_shelves(image_path)
#         print(shelves)
#     except Exception as e:
#         print(f"Error: {e}")
