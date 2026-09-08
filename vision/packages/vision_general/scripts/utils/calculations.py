#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped
import math
import sys

FLT_EPSILON = sys.float_info.epsilon


def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def get2DCentroid(box, frame):
    ymin = float(box[0])
    xmin = float(box[1])
    ymax = float(box[2])
    xmax = float(box[3])
    width = xmax - xmin
    height = ymax - ymin
    return (int(xmin + width / 2), int(ymin + height / 2))


def get2DCentroidNormalized(box, frame):
    ymin = float(box[0]) * frame.shape[0]
    xmin = float(box[1]) * frame.shape[1]
    ymax = float(box[2]) * frame.shape[0]
    xmax = float(box[3]) * frame.shape[1]
    width = xmax - xmin
    height = ymax - ymin
    return (int(xmin + width / 2), int(ymin + height / 2))


def get_depth(depthframe_, pixel):
    """
    Given pixel coordinates in an image, the actual image and its depth frame, compute the corresponding depth.
    """
    heightDEPTH, widthDEPTH = (depthframe_.shape[0], depthframe_.shape[1])

    x = pixel[1]
    y = pixel[0]

    def medianCalculation(x, y, width, height, depthframe_):
        medianArray = []
        requiredValidValues = 20

        def spiral(
            medianArray,
            depthframe_,
            requiredValidValues,
            startX,
            startY,
            endX,
            endY,
            width,
            height,
        ):
            if startX < 0 and startY < 0 and endX > width and endY > height:
                return
            # Check first and last row of the square spiral.
            startX, startY = int(startX), int(startY)
            endX, endY = int(endX), int(endY)
            for i in range(startX, endX + 1):
                if i >= width:
                    break
                if startY >= 0 and math.isfinite(depthframe_[startY][i]):
                    medianArray.append(depthframe_[startY][i])
                if (
                    startY != endY
                    and endY < height
                    and math.isfinite(depthframe_[endY][i])
                ):
                    medianArray.append(depthframe_[endY][i])
                if len(medianArray) > requiredValidValues:
                    return
            # Check first and last column of the square spiral.
            for i in range(startY + 1, endY):
                if i >= height:
                    break
                if startX >= 0 and math.isfinite(depthframe_[i][startX]):
                    medianArray.append(depthframe_[i][startX])
                if (
                    startX != endX
                    and endX < width
                    and math.isfinite(depthframe_[i][endX])
                ):
                    medianArray.append(depthframe_[i][endX])
                if len(medianArray) > requiredValidValues:
                    return
            # Go to the next outer square spiral of the depth pixel.
            spiral(
                medianArray,
                depthframe_,
                requiredValidValues,
                startX - 1,
                startY - 1,
                endX + 1,
                endY + 1,
                width,
                height,
            )

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
    """
    Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients,
    compute the corresponding point in 3D space relative to the same camera
    Reference: https://github.com/IntelRealSense/librealsense/blob/e9f05c55f88f6876633bd59fd1cb3848da64b699/src/rs.cpp#L3505
    """

    def CameraInfoToIntrinsics(cameraInfo):
        intrinsics = {}
        intrinsics["width"] = cameraInfo.width
        intrinsics["height"] = cameraInfo.height
        intrinsics["ppx"] = cameraInfo.k[2]
        intrinsics["ppy"] = cameraInfo.k[5]
        intrinsics["fx"] = cameraInfo.k[0]
        intrinsics["fy"] = cameraInfo.k[4]
        if cameraInfo.distortion_model == "plumb_bob":
            intrinsics["model"] = "RS2_DISTORTION_BROWN_CONRADY"
        elif cameraInfo.distortion_model == "equidistant":
            intrinsics["model"] = "RS2_DISTORTION_KANNALA_BRANDT4"
        elif cameraInfo.distortion_model == "rational_polynomial":
            intrinsics["model"] = "RS2_DISTORTION_MODIFIED_BROWN_CONRADY"
        intrinsics["coeffs"] = [i for i in cameraInfo.d]
        return intrinsics

    # Parse ROS CameraInfo msg to intrinsics dictionary.
    intrinsics = CameraInfoToIntrinsics(cv_image_rgb_info)

    x = (pixel[0] - intrinsics["ppx"]) / intrinsics["fx"]
    y = (pixel[1] - intrinsics["ppy"]) / intrinsics["fy"]

    xo = x
    yo = y

    if intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY":
        r2 = float(x * x + y * y)
        f = float(
            1
            + intrinsics["coeffs"][0] * r2
            + intrinsics["coeffs"][1] * r2 * r2
            + intrinsics["coeffs"][4] * r2 * r2 * r2
        )
        x *= f
        y *= f
        dx = (
            x
            + 2 * intrinsics["coeffs"][2] * x * y
            + intrinsics["coeffs"][3] * (r2 + 2 * x * x)
        )
        dy = (
            y
            + 2 * intrinsics["coeffs"][3] * x * y
            + intrinsics["coeffs"][2] * (r2 + 2 * y * y)
        )
        x = dx
        y = dy
    if intrinsics["model"] == "RS2_DISTORTION_INVERSE_BROWN_CONRADY":
        # need to loop until convergence
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(
                1
                + (
                    (intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2
                    + intrinsics["coeffs"][0]
                )
                * r2
            )
            xq = float(x / icdist)
            yq = float(y / icdist)
            delta_x = float(
                2 * intrinsics["coeffs"][2] * xq * yq
                + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq)
            )
            delta_y = float(
                2 * intrinsics["coeffs"][3] * xq * yq
                + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq)
            )
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
        # need to loop until convergence
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(
                1
                + (
                    (intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2
                    + intrinsics["coeffs"][0]
                )
                * r2
            )
            delta_x = float(
                2 * intrinsics["coeffs"][2] * x * y
                + intrinsics["coeffs"][3] * (r2 + 2 * x * x)
            )
            delta_y = float(
                2 * intrinsics["coeffs"][3] * x * y
                + intrinsics["coeffs"][2] * (r2 + 2 * y * y)
            )
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON

        theta = float(rd)
        theta2 = float(rd * rd)
        for i in range(4):
            f = float(
                theta
                * (
                    1
                    + theta2
                    * (
                        intrinsics["coeffs"][0]
                        + theta2
                        * (
                            intrinsics["coeffs"][1]
                            + theta2
                            * (
                                intrinsics["coeffs"][2]
                                + theta2 * intrinsics["coeffs"][3]
                            )
                        )
                    )
                )
                - rd
            )
            if math.fabs(f) < FLT_EPSILON:
                break
            df = float(
                1
                + theta2
                * (
                    3 * intrinsics["coeffs"][0]
                    + theta2
                    * (
                        5 * intrinsics["coeffs"][1]
                        + theta2
                        * (
                            7 * intrinsics["coeffs"][2]
                            + 9 * theta2 * intrinsics["coeffs"][3]
                        )
                    )
                )
            )
            theta -= f / df
            theta2 = theta * theta
        r = float(math.tan(theta))
        x *= r / rd
        y *= r / rd

    if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON
        r = (float)(
            math.tan(intrinsics["coeffs"][0] * rd)
            / math.atan(2 * math.tan(intrinsics["coeffs"][0] / float(2.0)))
        )
        x *= r / rd
        y *= r / rd

    return (depth * x, depth * y, depth)


def point2d_to_3d(
    image_info, depth_image, point2d: tuple[int, int]
) -> tuple[float, float, float]:
    """
    Given 2D pixel coordinates (x, y), intrinsic camera info and a depth image,
    calculates the proper depth and deprojects it into 3D Optical coordinates.
    """
    # get_depth expects (y, x) based on its implementation
    depth = get_depth(depth_image, (point2d[1], point2d[0]))
    # deproject_pixel_to_point expects (x, y)
    point3d = deproject_pixel_to_point(image_info, point2d, depth)
    return point3d


def point3d_to_ros_point_stamped(
    point3d: tuple[float, float, float], frame_id: str, stamp, is_optical: bool = True
) -> PointStamped:
    """
    Given a 3D point, transforms it into a standard ROS PointStamped.
    If is_optical=True (default), it converts from Camera Optical Frame (Z_forward, X_right, Y_down)
    to standard ROS base_link Frame (X_forward, Y_left, Z_up).
    """
    point_stamped = PointStamped()
    point_stamped.header.stamp = stamp
    point_stamped.header.frame_id = frame_id

    if is_optical:
        point_stamped.point.x = float(point3d[2])
        point_stamped.point.y = float(-point3d[0])
        point_stamped.point.z = float(-point3d[1])
    else:
        point_stamped.point.x = float(point3d[0])
        point_stamped.point.y = float(point3d[1])
        point_stamped.point.z = float(point3d[2])

    return point_stamped


def unrotate_pixel(
    pixel: tuple[int, int], rotation: int, width: int, height: int
) -> tuple[int, int]:
    """Inverse of cv2.rotate for valid rotations (0/90/180/270 deg)."""
    px, py = pixel
    rotation = int(rotation) % 360
    if rotation == 0:
        return px, py
    if rotation == 180:
        return width - 1 - px, height - 1 - py
    if rotation == 90:
        return py, height - 1 - px
    if rotation == 270:
        return width - 1 - py, px
    raise ValueError(f"Unsupported rotation {rotation}; expected 0/90/180/270")


def point2d_to_ros_point_stamped(
    image_info,
    depth_image,
    point2d: tuple[int, int],
    frame_id: str,
    stamp,
    is_optical: bool = False,
    rotation: int = 0,
) -> PointStamped:
    """
    Given 2D pixel coordinates (x, y), intrinsic camera info and a depth image,
    calculates the 3D position and directly returns a standard ROS PointStamped.
    The point is in the camera optical frame by default (is_optical=False means no extra conversion).
    Set is_optical=True only if you want to manually convert from optical to ROS base_link convention.

    If `rotation` is nonzero, `point2d` is un-rotated first so it indexes
    the raw depth/camera_info (use for pixels detected on an oriented image).
    """
    if rotation:
        point2d = unrotate_pixel(point2d, rotation, image_info.width, image_info.height)
    point_3d = point2d_to_3d(image_info, depth_image, point2d)
    return point3d_to_ros_point_stamped(point_3d, frame_id, stamp, is_optical)
