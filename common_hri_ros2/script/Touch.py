#!/usr/bin/env python3
# -*- coding: utf-8 -*-

######
#ROS2未対応
######

import os
import threading
import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rois_env_ros2.action import Execute
from rois_env_ros2.msg import Completed
from rois_env_ros2.srv import ComponentStatus, RobotAction


class TouchService(Node):
    def __init__(self, robotname, node_name):
        super().__init__(node_name)

        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Touch"
        self.robotname = robotname

        self.state = "idle"
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
        )
        print(exe_name)

        # Completed command publisher
        self.pub = self.create_publisher(
            Completed,
            self.robotname + "/completed_command",
            1,
        )

        # RobotAction service client
        config = get_robotfile()
        service_name = config["Param"]["RobotAction"]["service"]

        self.motion = self.create_client(
            RobotAction,
            service_name,
        )

        self.comp_state = "READY"
        self.get_logger().info(f"Component status: {self.comp_state}")

    def execute(self, goal_handle):
        self.comp_state = "BUSY"
        self.get_logger().info(f"Component status: {self.comp_state}")

        command = goal_handle.request.command_name
        self.get_logger().info(f"Received command: {command}")

        result = Execute.Result()

        if command == "start":
            self.start()
            result.success = "True"
            goal_handle.succeed()

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
        self.state = "moving"

        self.motion_thread = threading.Thread(
            target=self.monitor_playback,
            daemon=True,
        )
        self.motion_thread.start()

    def monitor_playback(self):
        self.get_logger().info("Monitoring playback.")

        try:
            if not self.motion.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("RobotAction service is not available.")

            request = RobotAction.Request()

            # 元コードどおり、TouchはRobotActionの"approach2"を呼ぶ
            request.action_name = "approach2"

            future = self.motion.call_async(request)

            # rclpy.spin() はメインスレッドで動作しているため、
            # このワーカースレッドではFutureの完了を待つ
            while rclpy.ok() and not future.done():
                threading.Event().wait(0.05)

            if not future.done():
                raise RuntimeError("RobotAction service call was interrupted.")

            service = future.result()

            if service is None:
                raise RuntimeError("RobotAction service returned no response.")

            print(service.success)

            if service.success:
                self.state = "OK"
            else:
                self.state = "ERROR"
                self.comp_state = "ERROR"

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            self.state = "ERROR"
            self.comp_state = "ERROR"
            self.get_logger().info(f"Component status: {self.comp_state}")

        finally:
            self.completed_command()

    def completed_command(self):
        self.get_logger().info("Playback completed.")

        pub_data = Completed()
        pub_data.command_id = "Touch"
        pub_data.status = self.state

        print(f"{pub_data.command_id}:{pub_data.status}")
        self.pub.publish(pub_data)

        if self.comp_state != "ERROR":
            self.comp_state = "READY"

        self.get_logger().info(f"Component status: {self.comp_state}")

    def stop(self):
        if self.state == "moving":
            # 実機側に停止APIがある場合はここに追加
            self.state = "stopped"

            pub_data = Completed()
            pub_data.command_id = "Touch"
            pub_data.status = "stopped"
            self.pub.publish(pub_data)

            self.comp_state = "READY"
            self.get_logger().info(f"Component status: {self.comp_state}")
            return True

        self.get_logger().warning("No active moving.")
        return False

    def suspend(self):
        if self.state == "moving":
            # 現状は論理状態のみ変更
            self.state = "suspended"
            return True

        self.get_logger().warning("Cannot suspend; not moving.")
        return False

    def resume(self):
        if self.state == "suspended":
            # 現状は論理状態のみ変更
            self.state = "moving"

            if not self.motion_thread or not self.motion_thread.is_alive():
                self.motion_thread = threading.Thread(
                    target=self.monitor_playback,
                    daemon=True,
                )
                self.motion_thread.start()

            return True

        self.get_logger().warning("No previous goal to resume.")
        return False

    def component_status(self, request, response):
        if request.component_name == "Touch":
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

    node_name = robot + "_Touch"
    service = TouchService(robotname, node_name)

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
