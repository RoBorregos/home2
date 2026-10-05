#!/usr/bin/env python3
"""Send N frames from a video file or webcam to the talking detection service.

Run INSIDE the vision container, with the workspace sourced and the node running
(`ros2 run vision_general talking_detection_node.py`):

    python3 /workspace/src/vision/scripts/test_talking.py                # webcam 0, 25 frames
    python3 /workspace/src/vision/scripts/test_talking.py clip.mp4 30    # video file, 30 frames

Full frames are sent, not bbox crops: the detector finds the face inside each one.
"""

import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from frida_constants.vision_constants import IS_TALKING_TOPIC
from frida_interfaces.srv import IsTalking
from rclpy.node import Node

DEFAULT_FRAMES = 25
SERVICE_WAIT_S = 5.0
CALL_TIMEOUT_S = 30.0


def read_frames(source, n_frames: int) -> list:
    bridge = CvBridge()
    cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
    frames = []
    while len(frames) < n_frames:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(bridge.cv2_to_imgmsg(frame, "bgr8"))
    cap.release()
    return frames


def main():
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    n_frames = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_FRAMES

    rclpy.init()
    node = Node("test_talking")
    client = node.create_client(IsTalking, IS_TALKING_TOPIC)
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
        sys.exit(f"service {IS_TALKING_TOPIC} not available")

    frames = read_frames(source, n_frames)
    if not frames:
        sys.exit(f"could not read any frames from {source!r}")
    print(f"sending {len(frames)} frames")

    request = IsTalking.Request()
    request.frames = frames
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_S)

    result = future.result()
    if result is None:
        print("no response")
    else:
        print(f"is_talking={result.is_talking} message={result.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
