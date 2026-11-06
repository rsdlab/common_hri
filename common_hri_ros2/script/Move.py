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
    MoveGetParam,
    MoveSetParam,
    MoveAction,
)


class MoveService(Node):
    """ROS2版 Move Component。ロボット依存処理は Move_action.py に任せる。"""

    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Move"
        self.robotname = robotname

        # 内部実行状態と最終結果は分離して管理する
        self.state = "idle"  # idle / moving / suspended
        self.completed_status = None

        self.line = []
        self.curve = []
        self.time = 0
        self.motion_thread = None

        print(self.comp_state)

        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = self.create_service(
            ComponentStatus,
            state_name,
            self.component_status,
        )
        self.get_logger().info("component status server is ready")

        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = ActionServer(
            self,
            Execute,
            exe_name,
            execute_callback=self.execute,
            callback_group=self.callback_group,
        )
        print(exe_name)

        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        config = get_robotfile()
        service_name = config["Param"]["MoveAction"]["service"]
        self.motion = self.create_client(
            MoveAction,
            service_name,
            callback_group=self.callback_group,
        )
        print(f"[DEBUG] MoveAction service_name = {service_name}")
        
        self.set_service = self.create_service(
            MoveSetParam,
            self.robotname + "/move_set_param",
            self.set_parameter_callback,
        )
        self.get_service = self.create_service(
            MoveGetParam,
            self.robotname + "/move_get_param",
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

    # ------------------------------------------------------------------
    # set / get parameter
    # ------------------------------------------------------------------

    def set_parameter_callback(self, request, response):
        self.get_logger().info("set parameter")

        try:
            line = list(request.line)
            curve = list(request.curve)
            time_value = request.time

            if line and len(line) < 3:
                raise ValueError(...)

            if curve and len(curve) < 2:
                raise ValueError(...)

            if not line and not curve:
                raise ValueError(...)

            # validation成功後
            self.line = line
            self.curve = curve
            self.time = time_value
            
            self.get_logger().info(f"line  = {self.line}")
            self.get_logger().info(f"curve = {self.curve}")
            self.get_logger().info(f"time  = {self.time} ms")

            response.returncode_t = "OK"
            response.command_id = "set_move"
            return response

        except Exception as e:
            self.get_logger().error(f"set_parameter failed: {e}")
            response.returncode_t = "BAD_PARAMETER"
            response.command_id = "set_move"
            return response

    def get_parameter_callback(self, request, response):
        self.get_logger().info("get_parameter")

        response.returncode_t = "OK"
        response.line = self.line
        response.curve = self.curve
        response.time = self.time
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
                f"Cannot start Move while component state is {self.comp_state}."
            )
            return False

        if not self.line and not self.curve:
            self.get_logger().warning("Move parameter is not set.")
            return False

        self.state = "moving"
        self.comp_state = "BUSY"
        self.completed_status = None
        self.get_logger().info(f"Component status: {self.comp_state}")

        self.motion_thread = threading.Thread(
            target=self.monitor_move,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_move(self):
        self.get_logger().info("Monitoring move.")

        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                self.completed_status = "TIMEOUT"
                self.state = "idle"
                self.comp_state = "ERROR"
                self.get_logger().error("MoveAction service wait timed out.")
                return

            request = MoveAction.Request()
            request.command = "start"
            request.line = self.line
            request.curve = self.curve
            request.time = self.time

            future = self.motion.call_async(request)

            while rclpy.ok() and not future.done():
                # stop成功時はstateがidleになる。
                # suspend中は処理未完了なので監視を継続する。
                if self.state == "idle":
                    return
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError("MoveAction service call was interrupted.")

            service = future.result()

            if service is None:
                raise RuntimeError(
                    "MoveAction service returned no response."
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
            self.get_logger().info(
                f"Component status: {self.comp_state}"
            )

        finally:
            if self.completed_status is not None:
                self.completed_command()

    def completed_command(self):
        self.get_logger().info("Move completed.")

        if self.completed_status is None:
            self.get_logger().warning(
                "Completed status is not set. Skip completed notification."
            )
            return

        pub_data = Completed()
        pub_data.command_id = "Move"
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

    # ------------------------------------------------------------------
    # stop / suspend / resume
    # ------------------------------------------------------------------

    def _send_control_command(self, command):
        request = MoveAction.Request()
        request.command = command
        request.line = []
        request.curve = []
        request.time = 0

        if not self.motion.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveAction service is not available.")
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error(f"Failed to {command} Move.")
            return False

        return True

    def stop(self):
        if self.state not in ("moving", "suspended"):
            self.get_logger().warning("No active Move operation.")
            return False

        if not self._send_control_command("stop"):
            return False

        return True

    def suspend(self):
        if self.state != "moving":
            self.get_logger().warning("Cannot suspend; not moving.")
            return False

        if not self._send_control_command("suspend"):
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            self.get_logger().warning("Cannot resume; not suspended.")
            return False

        if not self._send_control_command("resume"):
            return False

        self.state = "moving"
        return True

    # ------------------------------------------------------------------
    # Component status
    # ------------------------------------------------------------------

    def component_status(self, request, response):
        if request.component_name == self.comp_ref:
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

    node_name = robot + "_Move"
    service = MoveService(robotname, node_name)

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
