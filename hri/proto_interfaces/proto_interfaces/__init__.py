"""
Proto interfaces for HRI microservices.

This package provides gRPC service definitions and message types
for communication between ROS2 nodes and microservices.

Submodules:
    - speech_pb2: Speech-to-Text protocol buffer messages
    - speech_pb2_grpc: Speech-to-Text gRPC service definitions
    - tts_pb2: Text-to-Speech protocol buffer messages
    - tts_pb2_grpc: Text-to-Speech gRPC service definitions
"""

__version__ = "0.1.0"
__author__ = "RoBorregos"
__all__ = [
    "speech_pb2",
    "speech_pb2_grpc",
    "tts_pb2",
    "tts_pb2_grpc",
]
