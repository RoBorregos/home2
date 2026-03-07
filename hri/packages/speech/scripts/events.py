#!/usr/bin/env python3

import collections
import os
import sys
import time
import threading

import grpc
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from frida_interfaces.msg import AudioData
from frida_interfaces.srv import DetectEvent

sys.path.append(os.path.join(os.path.dirname(__file__), "events"))

import events_pb2
import events_pb2_grpc


class EventsNode(Node):
    def __init__(self):
        super().__init__("events_node")
        self.get_logger().info("*Starting Events Node*")

        server_ip = (
            self.declare_parameter("EVENTS_SERVER_IP", "127.0.0.1:50053")
            .get_parameter_value()
            .string_value
        )

        audio_topic = (
            self.declare_parameter("AUDIO_TOPIC", "/rawAudioChunk")
            .get_parameter_value()
            .string_value
        )

        self.service_name = (
            self.declare_parameter("DETECT_EVENT_SERVICE", "/hri/detect_event")
            .get_parameter_value()
            .string_value
        )

        self.stop_flag = threading.Event()
        self.stop_flag.set()
        self.first_event = threading.Event()
        self.detected_event = ""

        # gRPC Stub
        channel = grpc.insecure_channel(server_ip)
        self.stub = events_pb2_grpc.EventsServiceStub(channel)

        self.audio_buffer = collections.deque(maxlen=10000)

        subscription_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            AudioData,
            audio_topic,
            self.audio_callback,
            10,
            callback_group=subscription_group,
        )

        self.create_service(
            DetectEvent,
            self.service_name,
            self.service_callback,
            callback_group=service_group,
        )

        self.get_logger().info("*Events Node is ready*")

    def audio_callback(self, msg):
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.append(audio_data)

    def start_listening(self):
        self.get_logger().info("Events node listening.")
        call = None

        def request_generator():
            while not self.stop_flag.is_set() and rclpy.ok():
                try:
                    if not self.audio_buffer:
                        time.sleep(0.1)
                        continue
                    local_audio = self.audio_buffer.popleft()

                    if len(local_audio) < 10:
                        continue

                    grpc_audio = local_audio.tobytes()
                    yield events_pb2.AudioRequest(audio_data=grpc_audio)

                except IOError as e:
                    self.get_logger().error(f"I/O error({e.errno}): {e.strerror}")
                    break
            call.cancel()

        def handle_events(responses):
            try:
                for response in responses:
                    if self.stop_flag.is_set():
                        break
                    self.get_logger().info(f"Event detected: {response.text}")
                    self.detected_event = response.text
                    self.first_event.set()
                    break
            except grpc.RpcError as e:
                if "locally cancelled" not in e.details().lower():
                    self.get_logger().error(f"gRPC stream error: {e}")

        responses = self.stub.Listen(request_generator())
        call = responses
        listener = threading.Thread(target=handle_events, args=(responses,))
        listener.start()
        return listener

    def service_callback(self, request, response):
        self.get_logger().info("DetectEvent service called")

        self.stop_flag.clear()
        self.audio_buffer.clear()
        self.detected_event = ""
        self.first_event.clear()

        listener = self.start_listening()

        self.first_event.wait()

        self.stop_flag.set()
        listener.join()

        response.event = self.detected_event
        self.get_logger().info(f"Returning event: {response.event}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EventsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.stop_flag.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
