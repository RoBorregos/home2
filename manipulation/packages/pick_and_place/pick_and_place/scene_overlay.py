#!/usr/bin/env python3

"""Serve the live camera blended with a saved scene frame as an MJPEG page for rebuilding scenes."""

import argparse
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from frida_constants.vision_constants import CAMERA_TOPIC


class SceneOverlay(Node):
    def __init__(self, reference_path):
        super().__init__("scene_overlay")
        self.bridge = CvBridge()
        self.reference_path = reference_path
        self.reference = cv2.imread(reference_path)
        self.frame = None
        self.lock = threading.Lock()
        self.create_subscription(Image, CAMERA_TOPIC, self.on_image, 1)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.frame = frame

    def save_reference(self):
        """Store the current frame as the scene reference."""
        with self.lock:
            cv2.imwrite(self.reference_path, self.frame)
            self.reference = self.frame.copy()

    def jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            image = self.frame
            if self.reference is not None:
                image = cv2.addWeighted(self.frame, 0.5, self.reference, 0.5, 0)
            return cv2.imencode(".jpg", image)[1].tobytes()


def handler_for(overlay):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/save":
                overlay.save_reference()
                self.send_response(303)
                self.send_header("Location", "/")
                self.end_headers()
            elif self.path == "/":
                body = b'<a href="/save">Save reference</a><br><img src="/stream">'
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                while True:
                    data = overlay.jpeg()
                    if data is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(data + b"\r\n")
                    time.sleep(0.1)
            else:
                self.send_error(404)

    return Handler


def main():
    parser = argparse.ArgumentParser(description="Scene overlay")
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument("--dir", default="/workspace/src/bags/scenes")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()
    os.makedirs(args.dir, exist_ok=True)
    rclpy.init()
    overlay = SceneOverlay(os.path.join(args.dir, f"scene_{args.scene}.png"))
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(overlay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rclpy.spin(overlay)


if __name__ == "__main__":
    main()
