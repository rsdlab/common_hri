#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rois_env_ros2.action import Execute
from rois_env_ros2.msg import Completed
from rois_env_ros2.srv import (
    ApproachGetParam,
    ApproachSetParam,
    ComponentStatus,
    ApproachAction,
)


class ApproachService(Node):
    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Approach"
        self.robotname = robotname

        self.state = "idle"
        self.command = ""
        self.completed_status = None
        self.current_goal = None
        self.motion_thread = None

        print(self.comp_state)

        # Component status service
        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = self.create_service(
            ComponentStatus,
            state_name,
            self.component_status,
        )
        self.get_logger().info("component status server is ready")

        # Execute action server
        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = ActionServer(
            self,
            Execute,
            exe_name,
            execute_callback=self.execute,
            callback_group=self.callback_group,
        )
        print(exe_name)

        # Completed command publisher
        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        # ApproachAction service client
        config = get_robotfile()
        service_name = config["Param"]["ApproachAction"]["service"]
        self.motion = self.create_client(
            ApproachAction,
            service_name,
            callback_group=self.callback_group,
        )
        print(f"[DEBUG] ApproachAction service_name = {service_name}")

        # Approach parameter services
        self.set_service = self.create_service(
            ApproachSetParam,
            self.robotname + "/approach_set_param",
            self.set_parameter_callback,
        )
        self.get_service = self.create_service(
            ApproachGetParam,
            self.robotname + "/approach_get_param",
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

    def set_parameter_callback(self, request, response):
        print("set parameter")
        print(request.approach_command)

        command = request.approach_command

        if not command:
            response.set_return = "BAD_PARAMETER"
            return response

        self.command = command
        response.set_return = "OK"

        print(f"action_target：{self.command}")
        print("target_pose")

        return response

    def get_parameter_callback(self, request, response):
        print("get_parameter")
        print("action_target")
        print("target_pose")
        print(self.command)

        response.approach_command = self.command
        return response

    def execute(self, goal_handle):
        

        command = goal_handle.request.command_name
        self.get_logger().info(f"Received command: {command}")

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
            self.get_logger().info("No valid command received.")
            result.success = "False"
            goal_handle.abort()

        return result

    def start(self):
        if self.comp_state != "READY":
            self.get_logger().warning(
                f"Cannot start Approach while component state is {self.comp_state}."
            )
            return False
        
        if not self.command:
            self.get_logger().warning("Approach parameter is not set.")
            return False

        self.state = "moving"
        self.comp_state = "BUSY"
        self.get_logger().info(f"Component status: {self.comp_state}")

        self.motion_thread = threading.Thread(
            target=self.monitor_playback,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_playback(self):
        self.get_logger().info("Monitoring playback.")

        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                self.completed_status = "TIMEOUT"
                self.state = "idle"
                self.comp_state = "ERROR"
                self.get_logger().error("ApproachAction service wait timed out.")
                return

            request = ApproachAction.Request()
            request.command = "start"
            request.action_name = self.command

            future = self.motion.call_async(request)

            # rclpy.spin() はメインスレッドで動いているため、
            # ここではFutureの完了を待つだけにする
            while rclpy.ok() and not future.done():
                if self.state == "idle":
                    return
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError("ApproachAction service call was interrupted.")

            service = future.result()

            if service is None:
                raise RuntimeError(
                    "ApproachAction service returned no response."
                )

            self.completed_status = service.status
            self.state = "idle"

            if service.status in ("OK", "ABORT"):
                self.comp_state = "READY"
            else:
                self.comp_state = "ERROR"

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            self.completed_status = "ERROR"
            self.state = "idle"
            self.comp_state = "ERROR"
            self.get_logger().info(f"Component status: {self.comp_state}")

        finally:
            if self.completed_status is not None:
                self.completed_command()

    def completed_command(self):
        self.get_logger().info("Playback completed.")

        if self.completed_status is None:
            self.get_logger().warning(
                "Completed status is not set. Skip completed notification."
            )
            return

        pub_data = Completed()
        pub_data.command_id = "Approach"
        pub_data.status = self.completed_status

        print(f"{pub_data.command_id}:{pub_data.status}")
        self.pub.publish(pub_data)

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        self.get_logger().info(
            f"Component status: {self.comp_state}"
        )

        self.completed_status = None

    def stop(self):
        if self.state not in ("moving", "suspended"):
            self.get_logger().warning("No active Approach operation.")
            return False

        request = ApproachAction.Request()
        request.command = "stop"
        request.action_name = ""

        if not self.motion.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ApproachAction service is not available.")
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error("Failed to stop Approach.")
            return False

        return True

    def suspend(self):
        if self.state != "moving":
            self.get_logger().warning("Cannot suspend; not moving.")
            return False

        request = ApproachAction.Request()
        request.command = "suspend"
        request.action_name = ""

        if not self.motion.wait_for_service(timeout_sec=5.0):
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            self.get_logger().warning("Cannot resume; not suspended.")
            return False

        request = ApproachAction.Request()
        request.command = "resume"
        request.action_name = ""

        if not self.motion.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ApproachAction service is not available.")
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error("Failed to resume Approach.")
            return False

        self.state = "moving"
        return True


    def component_status(self, request, response):
        if request.component_name == "Approach":
            self.get_logger().info(
                f"Current state requested: {self.comp_state}"
            )
            response.status = self.comp_state

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
    robotname = "/" + robot
    print(f"Robot name: {robotname}")

    node_name = robot + "_Approach"
    service = ApproachService(robotname, node_name)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(service)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
