#!/usr/bin/env python3

import rclpy
import numpy as np
from collections import deque
from PIL import Image
from frida_interfaces.srv import EmbedText, EmbedCrops
from vision_runtime import VisionRuntime, spin
from frida_constants.vision_constants import (
    CLIP_EMBEDDING_MODEL,
    EMBED_TEXT_TOPIC,
    EMBED_CROPS_TOPIC,
    FINALS_BUFFER_SECONDS
)
from utils.models.image_embedder import ImageEmbedder
import threading

class FinalsCommandsNode(VisionRuntime):
    def __init__(self):
        super().__init__(
            "finals_commands_node",
        )

        self.embedder = ImageEmbedder(CLIP_EMBEDDING_MODEL).load()
        self.embedder_lock = threading.Lock()

        self.frames = deque()
        self.frames_lock = threading.Lock()

        self.create_service(
            EmbedText, EMBED_TEXT_TOPIC, self.embed_text_callback,
            callback_group=self.callback_group,
        )

        self.create_service(
            EmbedCrops, EMBED_CROPS_TOPIC, self.embed_crops_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Finals commands ready")


    def _image_callback(self, msg):
        super()._image_callback(msg)
        if self.image_header is not msg.header:
            return
        with self.frames_lock:
            self.frames.append((msg.header.stamp, self.image))
            while self.frames[0][0].sec < msg.header.stamp.sec - FINALS_BUFFER_SECONDS:
                self.frames.popleft()

    def embed_text_callback(self, request, response):
        response.dim = self.embedder.dim
        try:
            with self.embedder_lock:
                vectors = self.embedder.embed_text(list(request.texts), normalize=True)
        except Exception as e:
            self.get_logger().error(f"EmbedText failed: {e}")
            response.success = False
            return response
        response.embeddings = vectors.reshape(-1).tolist()
        response.success = True
        return response


    def embed_crops_callback(self, request, response):
        response.dim = self.embedder.dim
        with self.frames_lock:
            frame = next((img for s, img in self.frames if s == request.stamp), None)
        if frame is None:
            self.get_logger().warn("EmbedCrops: stamp not in buffer")
            response.success = False
            return response

        h, w = frame.shape[:2]
        crops = []
        for d in request.detections:
            x1, x2 = int(max(0.0, d.xmin) * w), int(min(1.0, d.xmax) * w)
            y1, y2 = int(max(0.0, d.ymin) * h), int(min(1.0, d.ymax) * h)
            if x2 <= x1 or y2 <= y1:
                self.get_logger().warn("EmbedCrops: empty box")
                response.success = False
                return response
            crops.append(Image.fromarray(np.ascontiguousarray(frame[y1:y2, x1:x2, ::-1])))

        with self.embedder_lock:
            vectors = self.embedder.embed_batch(crops, normalize=True)
        response.embeddings = vectors.reshape(-1).tolist()
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    spin(FinalsCommandsNode())


if __name__ == "__main__":
    main()
