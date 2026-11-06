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
    ObjectGraspingAction,
    ObjectGraspingGetParam,
    ObjectGraspingSetParam,
)


class ObjectGraspingService(Node):
    """RoIS Object Grasping component."""

    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.callback_group = ReentrantCallbackGroup()

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Object_Grasping"
        self.robotname = robotname

        self.object_id = ""
        self.grasp_force = 0.0

        # 内部状態と完了結果は分離
        self.state = "idle"  # idle / grasping / suspended
        self.completed_status = None
        self.motion_thread = None

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
            callback_group=self.callback_group,
        )

        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        config = get_robotfile()
        service_name = config["Param"]["ObjectGraspingAction"]["service"]
        self.motion = self.create_client(
            ObjectGraspingAction,
            service_name,
            callback_group=self.callback_group,
        )

        self.set_service = self.create_service(
            ObjectGraspingSetParam,
            self.robotname + "/object_grasping_set_param",
            self.set_parameter_callback,
        )
        self.get_service = self.create_service(
            ObjectGraspingGetParam,
            self.robotname + "/object_grasping_get_param",
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

    def set_parameter_callback(self, request, response):
        object_id = request.object_id.strip()
        grasp_force = float(request.grasp_force)

        if not object_id or grasp_force < 0.0:
            response.set_return = "BAD_PARAMETER"
            return response

        self.object_id = object_id
        self.grasp_force = grasp_force

        self.get_logger().info(
            f"Object Grasping parameters: object_id={self.object_id}, "
            f"grasp_force={self.grasp_force} N"
        )
        response.set_return = "OK"
        return response

    def get_parameter_callback(self, request, response):
        response.object_id = self.object_id
        response.grasp_force = self.grasp_force
        return response

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
                f"Cannot start Object_Grasping while component state is "
                f"{self.comp_state}."
            )
            return False

        if not self.object_id:
            self.get_logger().warning(
                "Object Grasping parameter is not set."
            )
            return False

        self.state = "grasping"
        self.completed_status = None
        self.comp_state = "BUSY"
        self.get_logger().info(f"Component status: {self.comp_state}")

        self.motion_thread = threading.Thread(
            target=self.monitor_grasping,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_grasping(self):
        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                self.completed_status = "TIMEOUT"
                self.state = "idle"
                self.comp_state = "ERROR"
                self.get_logger().error(
                    "ObjectGraspingAction service wait timed out."
                )
                return

            request = ObjectGraspingAction.Request()
            request.command = "start"
            request.object_id = self.object_id
            request.grasp_force = self.grasp_force

            future = self.motion.call_async(request)

            while rclpy.ok() and not future.done():
                if self.state == "idle":
                    return
                threading.Event().wait(0.05)


            if not future.done():
                raise RuntimeError(
                    "ObjectGraspingAction service call was interrupted."
                )

            service = future.result()

            if service is None:
                raise RuntimeError(
                    "ObjectGraspingAction service returned no response."
                )

            self.completed_status = service.status
            self.state = "idle"

            if service.status in ("OK", "ABORT"):
                self.comp_state = "READY"
            else:
                self.comp_state = "ERROR"
            

        except Exception as exc:
            self.get_logger().error(f"Object Grasping failed: {exc}")
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
            f"Object Grasping completed: {self.completed_status}"
        )

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        self.get_logger().info(
            f"Component status: {self.comp_state}"
        )

        self.completed_status = None

    def _send_control_command(self, command):
        request = ObjectGraspingAction.Request()
        request.command = command
        request.object_id = ""
        request.grasp_force = 0.0

        if not self.motion.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "ObjectGraspingAction service is not available."
            )
            return False

        future = self.motion.call_async(request)

        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error(
                f"Failed to {command} Object_Grasping."
            )
            return False

        return True

    def stop(self):
        if self.state not in ("grasping", "suspended"):
            self.get_logger().warning(
                "No active Object Grasping operation."
            )
            return False

        if not self._send_control_command("stop"):
            return False

        return True

    def suspend(self):
        if self.state != "grasping":
            self.get_logger().warning(
                "Cannot suspend Object Grasping; not grasping."
            )
            return False

        if not self._send_control_command("suspend"):
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            self.get_logger().warning(
                "Cannot resume Object Grasping; not suspended."
            )
            return False

        if not self._send_control_command("resume"):
            return False

        self.state = "grasping"
        return True

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
    service = ObjectGraspingService(
        "/" + robot,
        robot + "_Object_Grasping",
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
