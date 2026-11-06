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
from rois_env_ros2.srv import ComponentStatus, RobotAction


class LeaveService(Node):
    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Leave"
        self.robotname = robotname

        # 内部実行状態と完了結果を分離
        self.state = "idle"  # idle / moving / suspended
        self.completed_status = None
        self.motion_thread = None

        self.state_service = self.create_service(
            ComponentStatus,
            self.robotname + "/get_state/" + self.comp_ref,
            self.component_status,
        )
        self.get_logger().info("component status server is ready")

        self.server = ActionServer(
            self,
            Execute,
            self.robotname + "/execute/" + self.comp_ref,
            execute_callback=self.execute,
            callback_group=self.callback_group,
        )

        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        robot_data = get_robotfile()
        service_name = robot_data["Param"]["LeaveAction"]["service"]

        self.motion = self.create_client(
            RobotAction,
            service_name,
            callback_group=self.callback_group,
        )
        print(f"[DEBUG] LeaveAction service_name = {service_name}")

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

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
                f"Cannot start Leave while component state is {self.comp_state}."
            )
            return False

        self.state = "moving"
        self.completed_status = None
        self.comp_state = "BUSY"
        self.get_logger().info(f"Component status: {self.comp_state}")

        self.motion_thread = threading.Thread(
            target=self.monitor_leave,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_leave(self):
        self.get_logger().info("Monitoring Leave.")

        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                self.completed_status = "TIMEOUT"
                self.state = "idle"
                self.comp_state = "ERROR"
                self.get_logger().error(
                    "RobotAction service wait timed out."
                )
                return

            request = RobotAction.Request()
            request.action_name = "leave"

            future = self.motion.call_async(request)

            while rclpy.ok() and not future.done():
                if self.state == "idle":
                    return
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError(
                    "RobotAction service call was interrupted."
                )

            service = future.result()

            if service is None:
                raise RuntimeError(
                    "RobotAction service returned no response."
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
            f"Leave completed: {self.completed_status}"
        )

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        self.get_logger().info(
            f"Component status: {self.comp_state}"
        )

        self.completed_status = None

    def stop(self):
        # 現在のHSR Leave実装には、実行中のgo_absを安全に停止するための
        # 専用制御をまだ接続していないため未対応。
        if self.state not in ("moving", "suspended"):
            self.get_logger().warning("No active Leave operation.")
            return False

        self.get_logger().warning(
            "Leave stop is not supported by the current HSR implementation."
        )
        return False

    def suspend(self):
        if self.state != "moving":
            self.get_logger().warning(
                "Cannot suspend Leave; not moving."
            )
            return False

        self.get_logger().warning(
            "Leave suspend is not supported by the current HSR implementation."
        )
        return False

    def resume(self):
        if self.state != "suspended":
            self.get_logger().warning(
                "Cannot resume Leave; not suspended."
            )
            return False

        self.get_logger().warning(
            "Leave resume is not supported by the current HSR implementation."
        )
        return False

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

    service = LeaveService(
        robotname,
        robot + "_Leave",
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
