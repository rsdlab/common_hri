#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rois_env_ros2.action import Execute
from rois_env_ros2.msg import Completed
from rois_env_ros2.srv import (
    ComponentStatus,
    NaviGetParam,
    NaviSetParam,
    NavigationAction,
)


class NavigationService(Node):
    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Navigation"
        self.robotname = robotname

        # 内部実行状態と完了結果は分離
        self.state = "idle"  # idle / navigating / suspended
        self.completed_status = None

        self.target_position = []
        self.motion_thread = None

        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = self.create_service(
            ComponentStatus,
            state_name,
            self.component_status,
        )

        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = ActionServer(
            self,
            Execute,
            exe_name,
            execute_callback=self.execute,
            callback_group=self.callback_group,
        )

        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        config = get_robotfile()
        service_name = config["Param"]["NavigationAction"]["service"]
        self.motion = self.create_client(
            NavigationAction,
            service_name,
            callback_group=self.callback_group,
        )

        self.set_service = self.create_service(
            NaviSetParam,
            self.robotname + "/navi_set_param",
            self.set_parameter_callback,
        )
        self.get_service = self.create_service(
            NaviGetParam,
            self.robotname + "/navi_get_param",
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

    # ------------------------------------------------------------------
    # parameter
    # ------------------------------------------------------------------

    def set_parameter_callback(self, request, response):
        self.get_logger().info("set parameter")

        try:
            target_position = list(request.target_position)

            if len(target_position) == 0:
                raise ValueError("Navigation target is not set.")

            # validation成功後に保存
            self.target_position = target_position

            for i, target in enumerate(self.target_position):
                self.get_logger().info(
                    "target[%d]: position=(%.3f, %.3f, %.3f), "
                    "orientation=(%.3f, %.3f, %.3f, %.3f)"
                    % (
                        i,
                        target.x,
                        target.y,
                        target.z,
                        target.qx,
                        target.qy,
                        target.qz,
                        target.qw,
                    )
                )

            response.returncode_t = "OK"
            response.command_id = "set_navigation"
            return response

        except Exception as e:
            self.get_logger().error(f"set_parameter failed: {e}")
            response.returncode_t = "BAD_PARAMETER"
            response.command_id = "set_navigation"
            return response

    def get_parameter_callback(self, request, response):
        self.get_logger().info("get_parameter")

        response.returncode_t = "OK"
        response.target_position = self.target_position
        return response

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------

    def execute(self, goal_handle):
        command = goal_handle.request.command_name
        self.get_logger().info(f"Received command: {command}")

        result = Execute.Result()

        if command == "start":
            ok = self.start()
        elif command == "stop":
            ok = self.stop()
        elif command == "suspend":
            ok = self.suspend()
        elif command == "resume":
            ok = self.resume()
        else:
            self.get_logger().warning("No valid command received.")
            ok = False

        result.success = "True" if ok else "False"

        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def start(self):
        if self.comp_state != "READY":
            self.get_logger().warning(
                f"Cannot start Navigation while component state is "
                f"{self.comp_state}."
            )
            return False

        if len(self.target_position) == 0:
            self.get_logger().warning("Navigation target is not set.")
            return False

        self.state = "navigating"
        self.completed_status = None
        self.comp_state = "BUSY"
        self.get_logger().info(f"Component status: {self.comp_state}")

        self.motion_thread = threading.Thread(
            target=self.monitor_navigation,
            daemon=True,
        )
        self.motion_thread.start()

        return True

    def monitor_navigation(self):
        self.get_logger().info("Monitoring navigation.")

        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                self.completed_status = "TIMEOUT"
                self.state = "idle"
                self.comp_state = "ERROR"
                self.get_logger().error(
                    "NavigationAction service wait timed out."
                )
                return

            request = NavigationAction.Request()
            request.command = "start"
            request.target_position = self.target_position

            future = self.motion.call_async(request)

            while rclpy.ok() and not future.done():
                # stop成功時はidleになる。
                if self.state == "idle":
                    return
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError(
                    "NavigationAction service call was interrupted."
                )

            service = future.result()

            if service is None:
                raise RuntimeError(
                    "NavigationAction service returned no response."
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

        finally:
            if self.completed_status is not None:
                self.completed_command()

    def completed_command(self):
        if self.completed_status is None:
            self.get_logger().warning(
                "Completed status is not set. Skip completed notification."
            )
            return

        pub_data = Completed()
        pub_data.command_id = self.comp_ref
        pub_data.status = self.completed_status
        self.pub.publish(pub_data)

        self.get_logger().info(
            f"Navigation completed: {self.completed_status}"
        )

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        self.get_logger().info(
            f"Component status: {self.comp_state}"
        )

        self.completed_status = None

    # ------------------------------------------------------------------
    # stop / suspend / resume
    # ------------------------------------------------------------------

    def _send_control_command(self, command):
        request = NavigationAction.Request()
        request.command = command
        request.target_position = []

        if not self.motion.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "NavigationAction service is not available."
            )
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error(
                f"Failed to {command} Navigation."
            )
            return False

        return True

    def stop(self):
        if self.state not in ("navigating", "suspended"):
            self.get_logger().warning(
                "No active Navigation operation."
            )
            return False

        if not self._send_control_command("stop"):
            return False

        return True

    def suspend(self):
        # 下位実装が実際に一時停止に対応した場合のみ成功する。
        if self.state != "navigating":
            self.get_logger().warning(
                "Cannot suspend; not navigating."
            )
            return False

        if not self._send_control_command("suspend"):
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            self.get_logger().warning(
                "Cannot resume; not suspended."
            )
            return False

        if not self._send_control_command("resume"):
            return False

        self.state = "navigating"
        return True

    # ------------------------------------------------------------------
    # Component status
    # ------------------------------------------------------------------

    def component_status(self, request, response):
        if request.component_name == self.comp_ref:
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

    service = NavigationService(
        robotname,
        robot + "_Navigation",
    )

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
