#!/usr/bin/env python3
"""Publish a mic streamed over TCP (e.g. from a PC) as /hri/rawAudioChunk.

Replaces audio_capturer for tests: stop that node first or both publish.
Expects raw s16le, 16 kHz, mono (see ffmpeg command in the usage below).

PC:   ffmpeg -f dshow -i audio="<mic name>" -ac 1 -ar 16000 -f s16le tcp://<orin-ip>:5000
Orin: python3 pc_mic_bridge.py   (inside the hri-ros container)
"""

import socket

import rclpy
from frida_interfaces.msg import AudioData
from frida_constants.hri_constants import RAW_AUDIO_TOPIC

PORT = 5000
CHUNK_BYTES = 1024 * 2  # same 1024-sample chunks as audio_capturer


def main():
    rclpy.init()
    node = rclpy.create_node("pc_mic_bridge")
    pub = node.create_publisher(AudioData, RAW_AUDIO_TOPIC, 20)
    server = socket.create_server(("0.0.0.0", PORT))
    try:
        while rclpy.ok():
            node.get_logger().info(f"Waiting for the PC mic on port {PORT}...")
            conn, addr = server.accept()
            node.get_logger().info(f"PC mic connected from {addr[0]}")
            with conn:
                while rclpy.ok():
                    data = b""
                    while len(data) < CHUNK_BYTES:
                        part = conn.recv(CHUNK_BYTES - len(data))
                        if not part:
                            break
                        data += part
                    if len(data) < CHUNK_BYTES:
                        break  # stream ended; wait for a reconnect
                    pub.publish(AudioData(data=data))
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
