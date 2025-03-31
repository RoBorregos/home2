import sys
import math
import cv2 as cv
import numpy as np
def main(argv):

    THRESHOLD_MIN_VERTICAL = 20
    THRESHOLD_MAX_Y = 10
    THRESHOLD_MIN_HORIZONTAL = 50

    default_file = 'Shelf_Lab.jpg'
    filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    # RESIZE IMAGE  to 1280x720
    src = cv.resize(src, (1280, 720))
    
    dst = cv.Canny(src, 45, 302, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    
    
    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        # Horizontal line processing
        merged_horizontal_lines = []
        horizontal_line_lengths = []

        for i in range(0, len(linesP)):
            l = linesP[i][0]
            dx = abs(l[2] - l[0])
            dy = abs(l[3] - l[1])
            length = math.sqrt(dx**2 + dy**2)
            horizontal_line_lengths.append(length)

            # Check if the line is approximately horizontal
            if dy <10:  # Adjust the threshold as needed
                merged = False
                for j in range(len(merged_horizontal_lines)):
                    ml = merged_horizontal_lines[j]
                    if abs(l[1] - ml[1]) < THRESHOLD_MIN_HORIZONTAL and (abs(l[0] - ml[2]) < THRESHOLD_MIN_HORIZONTAL or abs(l[2] - ml[0]) < 100):
                        merged_horizontal_lines[j] = [min(ml[0], l[0]), ml[1], max(ml[2], l[2]), ml[3]]
                        merged = True
                        break
                if not merged:
                    merged_horizontal_lines.append(l)
        #Filter horizontal lines that are too short between them
        avg_horizontal_length = sum(horizontal_line_lengths) / len(horizontal_line_lengths)
        filtered_horizontal_lines = []
        for ml in merged_horizontal_lines:
            dx = abs(ml[2] - ml[0])
            dy = abs(ml[3] - ml[1])
            length = math.sqrt(dx**2 + dy**2)
            if length > avg_horizontal_length + 0.675*np.std(horizontal_line_lengths):
                filtered_horizontal_lines.append(ml)

        # Filter horizontal lines that are too close to each other
        final_horizontal_lines = []
        for i in range(len(filtered_horizontal_lines)):
            keep = True
            for j in range(len(final_horizontal_lines)):
                if abs(filtered_horizontal_lines[i][1] - final_horizontal_lines[j][1]) < 80:  # Adjust threshold as needed
                    keep = False
                    break
            if keep:
                final_horizontal_lines.append(filtered_horizontal_lines[i])

        #for fl in final_horizontal_lines:
            #cv.line(cdstP, (fl[0], fl[1]), (fl[2], fl[3]), (0, 0, 255), 3, cv.LINE_AA)

        # Vertical line processing
        merged_vertical_lines = []
        vertical_line_lengths = []

        for i in range(0, len(linesP)):
            l = linesP[i][0]
            dx = abs(l[2] - l[0])
            dy = abs(l[3] - l[1])
            length = math.sqrt(dx**2 + dy**2)
            vertical_line_lengths.append(length)

            # Check if the line is approximately vertical
            if dx < 50:  # Adjust the threshold as needed
                merged = False
                for j in range(len(merged_vertical_lines)):
                    ml = merged_vertical_lines[j]
                    if abs(l[0] - ml[0]) < THRESHOLD_MIN_VERTICAL and (abs(l[1] - ml[3]) < THRESHOLD_MIN_VERTICAL or abs(l[3] - ml[1]) < THRESHOLD_MAX_Y):
                        merged_vertical_lines[j] = [ml[0], min(ml[1], l[1]), ml[2], max(ml[3], l[3])]
                        merged = True
                        break
                if not merged:
                    merged_vertical_lines.append(l)

        filtered_vertical_lines = []

        # Define the minimum distance in the x-axis between the two longest vertical lines
        MIN_X_DISTANCE = 200  # Adjust this value as needed

        # Sort vertical lines by their length (y-axis) in descending order
        sorted_vertical_lines = sorted(merged_vertical_lines, key=lambda ml: abs(ml[3] - ml[1]), reverse=True)

        # Select the two longest vertical lines that are at least MIN_X_DISTANCE apart
        filtered_vertical_lines = []
        for i in range(len(sorted_vertical_lines)):
            for j in range(i + 1, len(sorted_vertical_lines)):
                x1 = sorted_vertical_lines[i][0]
                x2 = sorted_vertical_lines[j][0]
                if abs(x1 - x2) >= MIN_X_DISTANCE:
                    filtered_vertical_lines = [sorted_vertical_lines[i], sorted_vertical_lines[j]]
                    break
            if len(filtered_vertical_lines) == 2:
                break

        # Draw the selected vertical lines
        #for fl in filtered_vertical_lines:
            #cv.line(cdstP, (fl[0], fl[1]), (fl[2], fl[3]), (255, 0, 0), 3, cv.LINE_AA)

        # Draw shelf numbers
        if len(final_horizontal_lines) >= 2 and len(filtered_vertical_lines) == 2:
            # Sort horizontal lines by their y-coordinates (top to bottom)
            final_horizontal_lines = sorted(final_horizontal_lines, key=lambda l: l[1])

            # Get the x-coordinates of the two vertical lines
            x_left = filtered_vertical_lines[0][0]
            x_right = filtered_vertical_lines[1][0]

            # Iterate through each shelf (space between two horizontal lines)
            for i in range(len(final_horizontal_lines) - 1):
                y_top = final_horizontal_lines[i][1]
                y_bottom = final_horizontal_lines[i + 1][1]

                # Calculate the center of the shelf
                center_x = (x_left + x_right) // 2
                center_y = (y_top + y_bottom) // 2

                # Draw the shelf number
                shelf_number = i + 1
                cv.putText(cdstP, str(shelf_number), (center_x, center_y), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 2, cv.LINE_AA)
                cv.rectangle(cdstP, (x_left, y_top), (x_right, y_bottom), (0, 255, 0), 3)

    #cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.imshow("source", src)
    cv.imshow("output", cdstP)        

    cv.waitKey()
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])