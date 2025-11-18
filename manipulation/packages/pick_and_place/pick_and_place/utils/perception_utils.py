from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped


def get_object_point(object_name: str, detection_handler_client) -> PointStamped:
    request = DetectionHandler.Request()
    request.label = object_name
    request.closest_object = False
    detection_handler_client.wait_for_service()
    future = detection_handler_client.call_async(request)
    future = wait_for_future(future, timeout=2.0)
    point = PointStamped()
    if not future:
        return point

    if len(future.result().detection_array.detections) == 1:
        point = future.result().detection_array.detections[0].point3d
    elif len(future.result().detection_array.detections) > 1:
        closest_object = future.result().detection_array.detections[0]
        closest_distance = float("inf")
        for detection in future.result().detection_array.detections:
            distance = (
                detection.point3d.point.x**2
                + detection.point3d.point.y**2
                + detection.point3d.point.z**2
            ) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_object = detection
        point = closest_object.point3d

    return point


def get_object_cluster(point: PointStamped, perception_3d_client):
    request = PickPerceptionService.Request()
    request.point = point
    request.add_collision_objects = True
    perception_3d_client.wait_for_service()
    future = perception_3d_client.call_async(request)
    future = wait_for_future(future)

    pcl_result = future.result().cluster_result
    if len(pcl_result.data) == 0:
        return None
    return pcl_result
