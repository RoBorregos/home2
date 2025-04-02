import sys
import math
import cv2 as cv
import numpy as np


def detect_shelves(image_path):
    THRESHOLD_MIN_VERTICAL = 20
    THRESHOLD_MAX_Y = 10
    THRESHOLD_MIN_HORIZONTAL = 50
    MIN_X_DISTANCE = 200  # Minimum distance between vertical lines

    # Loads an image
    src = cv.imread(cv.samples.findFile(image_path), cv.IMREAD_GRAYSCALE)
    if src is None:
        raise ValueError(f"Error opening image: {image_path}")

    # Resize image to 1280x720
    src = cv.resize(src, (1280, 720))

    dst = cv.Canny(src, 45, 302, None, 3)

    # Copy edges to the images that will display the results in BGR
    # cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)

    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is None:
        return {}

    # Horizontal line processing
    merged_horizontal_lines = []
    horizontal_line_lengths = []

    for i in range(0, len(linesP)):
        line_segment = linesP[i][0]
        dx = abs(line_segment[2] - line_segment[0])
        dy = abs(line_segment[3] - line_segment[1])
        length = math.sqrt(dx**2 + dy**2)
        horizontal_line_lengths.append(length)

        if dy < 10:  # Check if the line is approximately horizontal
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

    avg_horizontal_length = sum(horizontal_line_lengths) / len(horizontal_line_lengths)
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
            if abs(filtered_horizontal_lines[i][1] - final_horizontal_lines[j][1]) < 80:
                keep = False
                break
        if keep:
            final_horizontal_lines.append(filtered_horizontal_lines[i])

    final_horizontal_lines = sorted(final_horizontal_lines, key=lambda line: line[1])

    # Vertical line processing
    merged_vertical_lines = []
    for i in range(0, len(linesP)):
        line_segment = linesP[i][0]
        dx = abs(line_segment[2] - line_segment[0])
        dy = abs(line_segment[3] - line_segment[1])

        if dx < 50:  # Check if the line is approximately vertical
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

    # Generate bounding boxes for shelves
    shelves = {}
    if len(final_horizontal_lines) >= 2 and len(filtered_vertical_lines) == 2:
        x_left = filtered_vertical_lines[0][0]
        x_right = filtered_vertical_lines[1][0]

        for i in range(len(final_horizontal_lines) - 1):
            y_top = final_horizontal_lines[i][1]
            y_bottom = final_horizontal_lines[i + 1][1]

            shelf_number = i + 1
            shelves[shelf_number] = {
                "x_left": x_left,
                "y_top": y_top,
                "x_right": x_right,
                "y_bottom": y_bottom,
            }

    return shelves


if __name__ == "__main__":
    try:
        image_path = sys.argv[1] if len(sys.argv) > 1 else "Shelf_Lab.jpg"
        shelves = detect_shelves(image_path)
        print(shelves)
    except Exception as e:
        print(f"Error: {e}")
