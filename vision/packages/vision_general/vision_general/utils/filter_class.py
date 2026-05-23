import rclpy
from builtin_interfaces.msg import Time
from tf2_geometry_msgs import do_transform_point

from frida_constants.vision_classes import BBOX
from vision_general.utils.calculations import point2d_to_ros_point_stamped
from vision_general.utils.area_check import is_point_in_room
from frida_constants.vision_constants import CAMERA_FRAME
from tf2_ros import Buffer

def filter_class(
    frame,
    detections: list,
    class_name: str,
    room_name: str,
    camera_info,
    depth_image,
    tf_buffer : Buffer,
    areas_json: dict,
    camera_rotation: int = 0,
) -> list:
    """
    Return only detections of class_name whose bbox center is inside room_name.

    class_name: int (YOLO class id, e.g. 0) or str (class label, e.g. 'person').
                Matched against BBOX.classname as a string.
    room_name:  key in areas_json polygon map (e.g. 'living_room').
    If spatial data (camera_info / depth_image / tf_buffer / areas_json) is
    unavailable, detections pass through without room filtering.
    """
    class_key = class_name if isinstance(class_name, str) else str(class_name)
    h, w = frame.shape[:2]
    filtered = []

    spatial_ready = (
        camera_info is not None
        and depth_image is not None
        and tf_buffer is not None
        and areas_json is not None
    )

    for det in detections:
        if det.classname != class_key:
            continue

        if not spatial_ready:
            filtered.append(det)
            continue

        cx = int((det.x1 + det.x2) / 2)
        cy = int((det.y1 + det.y2) / 2)

        if not (0 <= cx < w and 0 <= cy < h):
            continue

        point_stamped = point2d_to_ros_point_stamped(
            camera_info,
            depth_image,
            (cx, cy),
            CAMERA_FRAME,
            Time(sec=0, nanosec=0),
            rotation=camera_rotation,
        )

        try:
            transform = tf_buffer.lookup_transform(
                "map",
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            point_map = do_transform_point(point_stamped, transform)
        except Exception:
            continue

        if is_point_in_room(point_map, room_name, areas_json):
            filtered.append(det)

    return filtered
