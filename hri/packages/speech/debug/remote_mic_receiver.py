#!/usr/bin/env python3
"""Robot side: publish a remote mic stream (from remote_mic_sender.py) on /hri/rawAudioChunk.

Run inside the HRI container, with the real mic node stopped so the two don't interleave:
    pkill -f audio_capturer.py
    python3 remote_mic_receiver.py
Restart audio_capturer afterwards (relaunch the hri speech launch) to get the robot mic back.
"""

import socket

import rclpy
from frida_interfaces.msg import AudioData

PORT = 5005
CHUNK_BYTES = 2048  # 1024 int16 samples, same as audio_capturer.py

rclpy.init()
node = rclpy.create_node("remote_mic_receiver")
pub = node.create_publisher(AudioData, "/hri/rawAudioChunk", 20)

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", PORT))  # localhost only: reached through the ssh tunnel
srv.listen(1)
node.get_logger().info(f"Waiting for the laptop mic on 127.0.0.1:{PORT}")
try:
    while rclpy.ok():
        conn, _ = srv.accept()
        node.get_logger().info("Laptop mic connected")
        buf = b""
        with conn:
            while True:
                data = conn.recv(CHUNK_BYTES)
                if not data:
                    break
                buf += data
                while len(buf) >= CHUNK_BYTES:
                    pub.publish(AudioData(data=buf[:CHUNK_BYTES]))
                    buf = buf[CHUNK_BYTES:]
        node.get_logger().info("Laptop mic disconnected")
except KeyboardInterrupt:
    pass
finally:
    srv.close()
    rclpy.try_shutdown()
