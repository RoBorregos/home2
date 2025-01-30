#!/usr/bin/env python3

import math
import sys

FLT_EPSILON = sys.float_info.epsilon

def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get2DCentroid(box, frame):
    ymin = float(box[0]) * frame.shape[0]
    xmin = float(box[1]) * frame.shape[1]
    ymax = float(box[2]) * frame.shape[0]
    xmax = float(box[3]) * frame.shape[1]
    width = xmax - xmin
    height = ymax - ymin
    return (int(xmin + width / 2), int(ymin + height / 2))

def get_depth(depthframe_, pixel):
    '''
        Given pixel coordinates in an image, the actual image and its depth frame, compute the corresponding depth.
    '''
    heightDEPTH, widthDEPTH = (depthframe_.shape[0], depthframe_.shape[1])

    x = pixel[0]
    y = pixel[1]

    def medianCalculation(x, y, width, height, depthframe_):
        medianArray = []
        requiredValidValues = 20
        def spiral(medianArray, depthframe_, requiredValidValues, startX, startY, endX, endY, width, height):
            if startX <  0 and startY < 0 and endX > width and endY > height:
                return
            # Check first and last row of the square spiral.
            for i in range(startX, endX + 1):
                if i >= width:
                    break
                if startY >= 0 and math.isfinite(depthframe_[startY][i]):
                    medianArray.append(depthframe_[startY][i])
                if startY != endY and endY < height and math.isfinite(depthframe_[endY][i]):
                    medianArray.append(depthframe_[endY][i])
                if len(medianArray) > requiredValidValues:
                    return
            # Check first and last column of the square spiral.
            for i in range(startY + 1, endY):
                if i >= height:
                    break
                if startX >= 0 and math.isfinite(depthframe_[i][startX]):
                    medianArray.append(depthframe_[i][startX])
                if startX != endX and endX < width and math.isfinite(depthframe_[i][endX]):
                    medianArray.append(depthframe_[i][endX])
                if len(medianArray) > requiredValidValues:
                    return
            # Go to the next outer square spiral of the depth pixel.
            spiral(medianArray, depthframe_, requiredValidValues, startX - 1, startY - 1, endX + 1, endY + 1, width, height)
        
        # Check square spirals around the depth pixel till requiredValidValues found.
        spiral(medianArray, depthframe_, requiredValidValues, x, y, x, y, width, height)
        if len(medianArray) == 0:
            return float("NaN")

        # Calculate Median
        medianArray.sort()
        return medianArray[len(medianArray) // 2]
    
    # Get the median of the values around the depth pixel to avoid incorrect readings.
    return medianCalculation(x, y, widthDEPTH, heightDEPTH, depthframe_)

def deproject_pixel_to_point(cv_image_rgb_info, pixel, depth):
    '''
        Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, 
        compute the corresponding point in 3D space relative to the same camera
        Reference: https://github.com/IntelRealSense/librealsense/blob/e9f05c55f88f6876633bd59fd1cb3848da64b699/src/rs.cpp#L3505
    '''
    def CameraInfoToIntrinsics(cameraInfo):
        intrinsics = {}
        intrinsics["width"] = cameraInfo.width
        intrinsics["height"] = cameraInfo.height
        intrinsics["ppx"] = cameraInfo.K[2]
        intrinsics["ppy"] = cameraInfo.K[5]
        intrinsics["fx"] = cameraInfo.K[0]
        intrinsics["fy"] = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            intrinsics["model"] = "RS2_DISTORTION_BROWN_CONRADY"
        elif cameraInfo.distortion_model == 'equidistant':
            intrinsics["model"] = "RS2_DISTORTION_KANNALA_BRANDT4"
        intrinsics["coeffs"] = [i for i in cameraInfo.D]
        return intrinsics
    
    # Parse ROS CameraInfo msg to intrinsics dictionary.
    intrinsics = CameraInfoToIntrinsics(cv_image_rgb_info)

    if(intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY"): # Cannot deproject from a forward-distorted image
        return

    x = (pixel[0] - intrinsics["ppx"]) / intrinsics["fx"]
    y = (pixel[1] - intrinsics["ppy"]) / intrinsics["fy"]

    xo = x
    yo = y

    if (intrinsics["model"] == "RS2_DISTORTION_INVERSE_BROWN_CONRADY"):
        # need to loop until convergence 
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
            xq = float(x / icdist)
            yq = float(y / icdist)
            delta_x = float(2 * intrinsics["coeffs"][2] * xq * yq + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq))
            delta_y = float(2 * intrinsics["coeffs"][3] * xq * yq + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq))
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
        # need to loop until convergence 
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
            delta_x = float(2 * intrinsics["coeffs"][2] * x * y + intrinsics["coeffs"][3] * (r2 + 2 * x * x))
            delta_y = float(2 * intrinsics["coeffs"][3] * x * y + intrinsics["coeffs"][2] * (r2 + 2 * y * y))
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON

        theta = float(rd)
        theta2 = float(rd * rd)
        for i in range(4):
            f = float(theta * (1 + theta2 * (intrinsics["coeffs"][0] + theta2 * (intrinsics["coeffs"][1] + theta2 * (intrinsics["coeffs"][2] + theta2 * intrinsics["coeffs"][3])))) - rd)
            if fabs(f) < FLT_EPSILON:
                break
            df = float(1 + theta2 * (3 * intrinsics["coeffs"][0] + theta2 * (5 * intrinsics["coeffs"][1] + theta2 * (7 * intrinsics["coeffs"][2] + 9 * theta2 * intrinsics["coeffs"][3]))))
            theta -= f / df
            theta2 = theta * theta
        r = float(math.tan(theta))
        x *= r / rd
        y *= r / rd

    if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON
        r = (float)(math.tan(intrinsics["coeffs"][0] * rd) / math.atan(2 * math.tan(intrinsics["coeffs"][0] / float(2.0))))
        x *= r / rd
        y *= r / rd

    return (depth * x, depth * y, depth)

