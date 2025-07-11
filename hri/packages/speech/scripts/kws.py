#!/usr/bin/env python3

import os
import struct
from datetime import datetime

import pvporcupine
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import AudioData

ACCESS_KEY = os.getenv("ACCESS_KEY")


def list_files_with_extension(directory, extension):
    if not os.path.exists(directory):
        print(f"The directory '{directory}' does not exist.")
        return

    file_list = []

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))

    return file_list


class KeywordSpotting(Node):
    def __init__(self):
        super().__init__("keyword_spotting")
        self.get_logger().info("Initializing KeywordSpotting node.")

        # Ros parameters
        self.declare_parameter(
            "KEYWORD_DIR", "/workspace/src/hri/packages/speech/assets"
        )
        self.declare_parameter("audio_topic", "/rawAudioChunk")
        self.declare_parameter("WAKEWORD_TOPIC", "/keyword_detected")
        self.declare_parameter("sensitivity", 0.8)

        keyword_dir = (
            self.get_parameter("KEYWORD_DIR").get_parameter_value().string_value
        )

        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )

        wakeword_topic = (
            self.get_parameter("WAKEWORD_TOPIC").get_parameter_value().string_value
        )
        sensitivity = (
            self.get_parameter("sensitivity").get_parameter_value().double_value
        )

        # Properties
        keyword_paths = list_files_with_extension(keyword_dir, ".ppn")
        sensitivities = [sensitivity] * len(keyword_paths)

        try:
            self.porcupine = pvporcupine.create(
                access_key=ACCESS_KEY,
                keyword_paths=keyword_paths,
                sensitivities=sensitivities,
            )
            self.get_logger().info("KeywordSpotting working!")

        except pvporcupine.PorcupineActivationError as e:
            print("AccessKey activation error")
            raise e
        except pvporcupine.PorcupineActivationLimitError as e:
            print("AccessKey '%s' has reached it's temporary device limit" % ACCESS_KEY)
            raise e
        except pvporcupine.PorcupineActivationRefusedError as e:
            print("AccessKey '%s' refused" % ACCESS_KEY)
            raise e
        except pvporcupine.PorcupineActivationThrottledError as e:
            print("AccessKey '%s' has been throttled" % ACCESS_KEY)
            raise e
        except pvporcupine.PorcupineError as e:
            print("Failed to initialize Porcupine")
            raise e

        self.keywords = list()
        for x in keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace(".ppn", "").split("_")
            if len(keyword_phrase_part) > 6:
                self.keywords.append(" ".join(keyword_phrase_part[0:-6]))
            else:
                self.keywords.append(keyword_phrase_part[0])

        # Ros interactions
        self.publisher = self.create_publisher(String, wakeword_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.detect_keyword, 10)

        self.get_logger().info("KeywordSpotting node initialized.")

    def get_next_audio_frame(self, msg):
        pcm = msg.data
        pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)
        return pcm

    def detect_keyword(self, msg):
        audio_frame = self.get_next_audio_frame(msg)
        result = self.porcupine.process(audio_frame)
        # self.get_logger().info("attemtping kws inference")
        if result >= 0:
            self.get_logger().info(
                "[%s] Detected %s" % (str(datetime.now()), self.keywords[result])
            )
            self.publisher.publish(String(data="frida"))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KeywordSpotting())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
