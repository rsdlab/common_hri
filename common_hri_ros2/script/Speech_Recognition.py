#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
from datetime import datetime

import yaml
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rois_env_ros2.action import Execute
from rois_env_ros2.msg import NotifySpeechrec
from rois_env_ros2.srv import (
    ComponentStatus,
    SpeechRecognitionGetParam,
    SpeechRecognitionSetParam,
    SpeechRecognitionAction,
)


class SpeechRecognitionService(Node):

    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Speech_Recognition"
        self.robotname = robotname

        self.recognizable_list = [
            "japanese",
            "english",
            "french",
        ]

        self.language_map = {
            "japanese": "ja-JP",
            "english": "en-US",
            "french": "fr-FR",
        }

        self.set_language = "japanese"
        self.languages = "ja-JP"
        self.recognized_text = ""

        self.recognition_thread = None
        self.state = "idle"

        self.state_service = self.create_service(
            ComponentStatus,
            self.robotname + "/get_state/" + self.comp_ref,
            self.component_status,
        )

        self.server = ActionServer(
            self,
            Execute,
            self.robotname + "/execute/" + self.comp_ref,
            execute_callback=self.execute,
        )

        self.set_service = self.create_service(
            SpeechRecognitionSetParam,
            self.robotname + "/s_recognition_set_param",
            self.set_parameter_callback,
        )

        self.get_service = self.create_service(
            SpeechRecognitionGetParam,
            self.robotname + "/s_recognition_get_param",
            self.get_parameter_callback,
        )

        self.pub = self.create_publisher(
            NotifySpeechrec,
            self.robotname + "/event_speechrec",
            1,
        )

        config = get_robotfile()

        service_name = config["Param"]["SpeechRecognitionAction"]["service"]

        self.recognition_client = self.create_client(
            SpeechRecognitionAction,
            service_name,
        )

        self.comp_state = "READY"

        self.get_logger().info(
            f"Speech_Recognition ready. implementation={service_name}"
        )

    def set_parameter_callback(self, request, response):
        language = request.languages.strip().lower()

        if language not in self.recognizable_list:
            self.get_logger().warning(
                f"Unsupported language: {language}"
            )

            response.set_return = "BAD_PARAMETER"
            return response

        self.set_language = language
        self.languages = self.language_map[language]

        response.set_return = "OK"

        self.get_logger().info(
            f"Speech recognition language set: "
            f"{self.set_language} ({self.languages})"
        )

        return response

    def get_parameter_callback(self, request, response):
        response.languages = self.set_language
        return response

    def execute(self, goal_handle):
        self.comp_state = "BUSY"

        command = goal_handle.request.command_name
        result = Execute.Result()

        if command == "start":
            if self.start():
                result.success = "True"
                goal_handle.succeed()
            else:
                result.success = "False"
                goal_handle.abort()

        elif command == "stop":
            if self.stop():
                result.success = "True"
                goal_handle.succeed()
            else:
                result.success = "False"
                goal_handle.abort()

        elif command == "suspend":
            if self.suspend():
                result.success = "True"
                goal_handle.succeed()
            else:
                result.success = "False"
                goal_handle.abort()

        elif command == "resume":
            if self.resume():
                result.success = "True"
                goal_handle.succeed()
            else:
                result.success = "False"
                goal_handle.abort()

        else:
            self.get_logger().warning(
                f"Unsupported command: {command}"
            )

            self.comp_state = "READY"

            result.success = "False"
            goal_handle.abort()

        return result

    def start(self):
        if (
            self.recognition_thread
            and self.recognition_thread.is_alive()
        ):
            self.get_logger().warning(
                "Speech recognition is already running."
            )

            self.comp_state = "BUSY"
            return False

        self.state = "recognizing"

        self.recognition_thread = threading.Thread(
            target=self.recognize,
            daemon=True,
        )

        self.recognition_thread.start()

        return True

    def recognize(self):
        try:
            if not self.recognition_client.wait_for_service(
                timeout_sec=5.0
            ):
                raise RuntimeError(
                    "SpeechRecognitionAction service "
                    "is not available."
                )

            request = SpeechRecognitionAction.Request()
            request.language = self.languages

            self.get_logger().info(
                f"Start speech recognition: {self.languages}"
            )

            future = self.recognition_client.call_async(
                request
            )

            while rclpy.ok() and not future.done():
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError(
                    "SpeechRecognitionAction service call "
                    "was interrupted."
                )

            response = future.result()

            if response is None:
                raise RuntimeError(
                    "SpeechRecognitionAction returned "
                    "no response."
                )

            if not response.success:
                raise RuntimeError(
                    "Speech recognition failed."
                )

            self.recognized_text = (
                response.recognized_text
            )

            self.get_logger().info(
                f"Recognized text: "
                f"{self.recognized_text}"
            )

            self.publish_recognition_result(
                self.recognized_text
            )

            self.state = "OK"
            self.comp_state = "READY"

        except Exception as exc:
            self.get_logger().error(
                f"Speech Recognition failed: {exc}"
            )

            self.state = "ERROR"
            self.comp_state = "ERROR"

    def publish_recognition_result(self, text):
        msg = NotifySpeechrec()

        msg.event_type = "speech_recognized"
        msg.timestamp = (
            datetime.now().isoformat()
        )
        msg.recognized_text = text

        self.pub.publish(msg)

    def stop(self):
        self.get_logger().warning(
            "stop is not supported by the current "
            "SpeechRecognitionAction interface."
        )

        self.comp_state = (
            "BUSY"
            if self.state == "recognizing"
            else "READY"
        )

        return False

    def suspend(self):
        self.get_logger().warning(
            "suspend is not supported by the current "
            "SpeechRecognitionAction interface."
        )

        self.comp_state = (
            "BUSY"
            if self.state == "recognizing"
            else "READY"
        )

        return False

    def resume(self):
        self.get_logger().warning(
            "resume is not supported by the current "
            "SpeechRecognitionAction interface."
        )

        self.comp_state = (
            "BUSY"
            if self.state == "recognizing"
            else "READY"
        )

        return False

    def component_status(
        self,
        request,
        response,
    ):
        if request.component_name == self.comp_ref:
            response.status = self.comp_state
        else:
            response.status = "UNKNOWN_COMPONENT"

        return response

    def destroy_node(self):
        self.server.destroy()
        super().destroy_node()


def get_robotfile():
    path = os.path.join(
        get_package_share_directory(
            "common_hri_ros2"
        ),
        "robot.yaml",
    )

    with open(
        path,
        "r",
        encoding="utf-8",
    ) as file:
        return yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)

    robot = get_robotfile()["Robot"]

    service = SpeechRecognitionService(
        "/" + robot,
        robot + "_Speech_Recognition",
    )

    try:
        rclpy.spin(service)

    except KeyboardInterrupt:
        pass

    finally:
        service.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()