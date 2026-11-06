#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading

import yaml
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rois_env_ros2.action import Execute
from rois_env_ros2.msg import Completed
from rois_env_ros2.srv import (
    ComponentStatus,
    SpeechGetParam,
    SpeechSetParam,
    SpeechAction,
)


class SpeechSynthesisService(Node):
    """Robot-independent RoIS Speech Synthesis component."""

    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Speech_Synthesis"
        self.robotname = robotname

        self.text = ""
        self.state = "idle"
        self.playback_thread = None

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
            SpeechSetParam,
            self.robotname + "/speech_set_param",
            self.set_parameter_callback,
        )

        self.get_service = self.create_service(
            SpeechGetParam,
            self.robotname + "/speech_get_param",
            self.get_parameter_callback,
        )

        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        config = get_robotfile()
        service_name = config["Param"]["SpeechSynthesisAction"]["service"]

        self.speech_client = self.create_client(
            SpeechAction,
            service_name,
        )

        self.comp_state = "READY"

        self.get_logger().info(
            f"Speech_Synthesis ready. implementation={service_name}"
        )

    def set_parameter_callback(self, request, response):
        speech_text = request.speech_text.strip()

        if not speech_text:
            response.set_return = "BAD_PARAMETER"
            self.get_logger().warning("speech_text is empty.")
            return response

        self.text = speech_text
        response.set_return = "OK"

        self.get_logger().info(
            f"Speech text set: {self.text}"
        )

        return response

    def get_parameter_callback(self, request, response):
        response.speech_text = self.text
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
        if not self.text:
            self.get_logger().warning(
                "Speech Synthesis parameter is not set."
            )

            self.comp_state = "READY"
            return False

        if self.playback_thread and self.playback_thread.is_alive():
            self.get_logger().warning(
                "Speech Synthesis is already running."
            )

            self.comp_state = "BUSY"
            return False

        self.state = "playing"

        self.playback_thread = threading.Thread(
            target=self.monitor_playback,
            daemon=True,
        )

        self.playback_thread.start()

        return True

    def monitor_playback(self):
        try:
            if not self.speech_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    "SpeechAction service is not available."
                )

            request = SpeechAction.Request()
            request.speech_text = self.text

            future = self.speech_client.call_async(request)

            while rclpy.ok() and not future.done():
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError(
                    "SpeechAction service call was interrupted."
                )

            response = future.result()

            if response is None:
                raise RuntimeError(
                    "SpeechAction returned no response."
                )

            if response.success:
                self.state = "OK"
            else:
                self.state = "ERROR"
                self.comp_state = "ERROR"

        except Exception as exc:
            self.get_logger().error(
                f"Speech Synthesis failed: {exc}"
            )

            self.state = "ERROR"
            self.comp_state = "ERROR"

        finally:
            self.completed_command()

    def completed_command(self):
        pub_data = Completed()

        pub_data.command_id = self.comp_ref
        pub_data.status = self.state

        self.pub.publish(pub_data)

        if self.comp_state != "ERROR":
            self.comp_state = "READY"

        self.get_logger().info(
            f"Speech Synthesis completed: {self.state}"
        )

    def stop(self):
        self.get_logger().warning(
            "stop is not supported by the current "
            "SpeechAction interface."
        )

        self.comp_state = (
            "READY" if self.state != "playing" else "BUSY"
        )

        return False

    def suspend(self):
        self.get_logger().warning(
            "suspend is not supported by the current "
            "SpeechAction interface."
        )

        self.comp_state = (
            "READY" if self.state != "playing" else "BUSY"
        )

        return False

    def resume(self):
        self.get_logger().warning(
            "resume is not supported by the current "
            "SpeechAction interface."
        )

        self.comp_state = (
            "READY" if self.state != "playing" else "BUSY"
        )

        return False

    def component_status(self, request, response):
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
        get_package_share_directory("common_hri_ros2"),
        "robot.yaml",
    )

    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)

    robot = get_robotfile()["Robot"]

    service = SpeechSynthesisService(
        "/" + robot,
        robot + "_Speech_Synthesis",
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