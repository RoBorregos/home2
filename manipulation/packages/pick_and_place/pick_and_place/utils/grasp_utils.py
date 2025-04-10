import copy
from geometry_msgs.msg import PointStamped, PoseStamped
from frida_interfaces.srv import GraspDetection
from frida_motion_planning.utils.ros_utils import wait_for_future


def get_grasps(grasp_detection_client, object_cloud, cgf_path: str):
    request = GraspDetection.Request()
    request.input_cloud = object_cloud
    request.cfg_path = cgf_path
    grasp_detection_client.wait_for_service()
    future = grasp_detection_client.call_async(request)
    future = wait_for_future(future)

    response = future.result()
    if len(response.grasp_poses) == 0:
        return []
    return response.grasp_poses, response.grasp_scores


def fake_grasps(object_point: PointStamped):
    grasp_pose1 = PoseStamped()
    grasp_pose1.header.frame_id = object_point.header.frame_id
    grasp_pose1.pose.position.x = object_point.point.x
    grasp_pose1.pose.position.y = object_point.point.y
    grasp_pose1.pose.position.z = object_point.point.z + 0.10
    grasp_pose1.pose.orientation.x = 1.0
    grasp_pose1.pose.orientation.y = 0.0
    grasp_pose1.pose.orientation.z = 0.0
    grasp_pose1.pose.orientation.w = 0.0

    grasp_pose2 = copy.deepcopy(grasp_pose1)
    grasp_pose2.pose.position.z = object_point.point.z + 0.25

    grasp_poses = [grasp_pose1, grasp_pose2]
    grasp_scores = [0.9, 0.8]

    return grasp_poses, grasp_scores
