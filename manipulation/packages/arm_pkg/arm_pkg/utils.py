import colorsys
import numpy as np
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

DEFAULT_GRIPPER_DIMENSIONS = {
    "base": (0.1, 0.1, 0.02),
    "finger": (0.02, 0.02, 0.17),
    "separation": 0.08,
}

def create_marker(
    frame_id,
    ns,
    marker_id,
    marker_type,
    position,
    orientation,
    scale,
    color,
    timestamp=None
):
    """
    Create a visualization Marker with specified parameters
    
    Args:
        frame_id (str): TF frame ID
        ns (str): Namespace for the marker
        marker_id (int): Unique ID for the marker
        marker_type (int): Marker type constant from visualization_msgs.msg.Marker
        position (tuple): (x, y, z) position
        orientation (tuple): (x, y, z, w) quaternion
        scale (tuple): (x, y, z) scale
        color (tuple): (r, g, b, a) color values 0-1
        timestamp (rospy.Time): Optional timestamp
    
    Returns:
        visualization_msgs.msg.Marker
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    if timestamp:
        marker.header.stamp = timestamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

def create_gripper(
    base_pose,
    frame_id="base_link",
    ns="gripper",
    hue=0.5,
    dimensions=None,
    timestamp=None
):
    """
    Create a gripper visualization MarkerArray
    
    Args:
        base_pose (geometry_msgs.msg.Pose): Base pose of the gripper
        frame_id (str): TF frame ID
        ns (str): Namespace for the markers
        hue (float): Color hue (0-1)
        dimensions (dict): Gripper dimensions dictionary with keys:
            - base: (x, y, z) base dimensions
            - finger: (x, y, z) finger dimensions
            - separation: float between fingers
        timestamp (rospy.Time): Optional timestamp
    
    Returns:
        visualization_msgs.msg.MarkerArray
    """
    # Set default dimensions if not provided
    dimensions = dimensions or DEFAULT_GRIPPER_DIMENSIONS.copy()
    
    markers = []
    r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

    # Create transformation matrix
    trans = tf_transformations.translation_matrix([
        base_pose.position.x,
        base_pose.position.y, 
        base_pose.position.z
    ])
    rot = tf_transformations.quaternion_matrix([
        base_pose.orientation.x,
        base_pose.orientation.y,
        base_pose.orientation.z,
        base_pose.orientation.w
    ])
    yaw_rot = tf_transformations.euler_matrix(0, 0, np.pi/2)
    transform = trans @ rot @ yaw_rot

    # Base marker
    translation = tf_transformations.translation_from_matrix(transform)
    base_quat = tf_transformations.quaternion_from_matrix(transform)
    
    base_marker = create_marker(
        frame_id=frame_id,
        ns=ns,
        marker_id=0,
        marker_type=Marker.CUBE,
        position=translation,
        orientation=base_quat,
        scale=dimensions["base"],
        color=(r * 0.5, g * 0.5, b * 0.5, 0.8),
        timestamp=timestamp
    )
    markers.append(base_marker)

    # Finger markers
    for i in range(2):
        x_offset = ((-1) ** i) * dimensions["separation"] / 2
        finger_transform = transform @ tf_transformations.translation_matrix(
            [x_offset, 0, dimensions["finger"][2] / 2]
        )

        finger_marker = create_marker(
            frame_id=frame_id,
            ns=ns,
            marker_id=i+1,
            marker_type=Marker.CUBE,
            position=tf_transformations.translation_from_matrix(finger_transform),
            orientation=base_quat,
            scale=dimensions["finger"],
            color=(r, g, b, 0.8),
            timestamp=timestamp
        )
        markers.append(finger_marker)

    return MarkerArray(markers=markers)