#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS1 common Move component.

MainEngine
  -> /<Robot>/execute/Move (Execute action)
  -> /<Robot>/move_set_param, /move_get_param
common_hri/Move.py
  -> robot-specific MoveAction service

No odom/cmd_vel/PI control exists here; those belong to *_hri_action.
"""

import os
import threading

import actionlib
import rospy
import rospkg
import yaml

from rois_env_ros1.msg import Completed
from rois_env_ros1.msg import ExecuteAction, ExecuteResult
from rois_env_ros1.srv import (
    ComponentStatus,
    ComponentStatusResponse,
    MoveAction,
    MoveActionRequest,
    MoveGetParam,
    MoveGetParamResponse,
    MoveSetParam,
    MoveSetParamResponse,
)


class MoveService:
    def __init__(self, robotname):
        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Move"
        self.robotname = robotname

        self.state = "idle"
        self.line = []
        self.curve = []
        self.time = 0
        self.completed_status = None
        self.motion_thread = None

        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = rospy.Service(
            state_name,
            ComponentStatus,
            self.component_status,
        )
        rospy.loginfo("Move component status server is ready: %s", state_name)

        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = actionlib.SimpleActionServer(
            exe_name,
            ExecuteAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("Move action server is ready: %s", exe_name)

        self.pub = rospy.Publisher(
            self.robotname + "/completed_command",
            Completed,
            queue_size=1,
        )

        config = get_robotfile()
        self.motion_service_name = config["Param"]["MoveAction"]["service"]
        self.motion = rospy.ServiceProxy(
            self.motion_service_name,
            MoveAction,
        )
        rospy.loginfo(
            "MoveAction client configured: %s",
            self.motion_service_name,
        )

        self.set_service = rospy.Service(
            self.robotname + "/move_set_param",
            MoveSetParam,
            self.set_parameter_callback,
        )
        self.get_service = rospy.Service(
            self.robotname + "/move_get_param",
            MoveGetParam,
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        rospy.loginfo("Move Component status: %s", self.comp_state)

    # ------------------------------------------------------------------
    # Parameter
    # ------------------------------------------------------------------
    def set_parameter_callback(self, request):
        response = MoveSetParamResponse()

        try:
            line = list(request.line)
            curve = list(request.curve)
            time_value = request.time

            if line and len(line) < 3:
                raise ValueError(
                    "line must contain [x_mm, y_mm, yaw_deg]"
                )

            if curve and len(curve) < 2:
                raise ValueError(
                    "curve must contain [radius_mm, angle_deg]"
                )

            if not line and not curve:
                raise ValueError("Either line or curve must be specified.")

            self.line = line
            self.curve = curve
            self.time = time_value

            rospy.loginfo("Move line  = %s", self.line)
            rospy.loginfo("Move curve = %s", self.curve)
            rospy.loginfo("Move time  = %s ms", self.time)

            response.returncode_t = "OK"
            response.command_id = "set_move"

        except Exception as exc:
            rospy.logerr("Move set_parameter failed: %s", exc)
            response.returncode_t = "BAD_PARAMETER"
            response.command_id = "set_move"

        return response

    def get_parameter_callback(self, request):
        response = MoveGetParamResponse()
        response.returncode_t = "OK"
        response.line = self.line
        response.curve = self.curve
        response.time = self.time
        return response

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------
    def execute(self, goal):
        command = goal.command_name
        rospy.loginfo("Move received command: %s", command)

        result = ExecuteResult()

        if command == "start":
            ok = self.start()
        elif command == "stop":
            ok = self.stop()
        elif command == "suspend":
            ok = self.suspend()
        elif command == "resume":
            ok = self.resume()
        else:
            rospy.logwarn("Unknown Move command: %s", command)
            ok = False

        result.success = "True" if ok else "False"

        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if self.comp_state != "READY":
            rospy.logwarn(
                "Cannot start Move while component state is %s",
                self.comp_state,
            )
            return False

        if not self.line and not self.curve:
            rospy.logwarn("Move parameter is not set.")
            return False

        self.state = "moving"
        self.comp_state = "BUSY"
        self.completed_status = None

        rospy.loginfo("Move Component status: %s", self.comp_state)

        self.motion_thread = threading.Thread(
            target=self.monitor_move,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_move(self):
        rospy.loginfo("Monitoring Move.")

        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)

            request = MoveActionRequest()
            request.command = "start"
            request.line = self.line
            request.curve = self.curve
            request.time = self.time

            service = self.motion(request)

            self.completed_status = service.status
            self.state = "idle"

            if service.status in ("OK", "ABORT"):
                self.comp_state = "READY"
            else:
                self.comp_state = "ERROR"

        except rospy.ROSException:
            rospy.logerr("MoveAction service wait timed out.")
            self.completed_status = "TIMEOUT"
            self.state = "idle"
            self.comp_state = "ERROR"

        except rospy.ServiceException as exc:
            rospy.logerr("MoveAction service call failed: %s", exc)
            self.completed_status = "ERROR"
            self.state = "idle"
            self.comp_state = "ERROR"

        except Exception as exc:
            rospy.logerr("Unexpected Move error: %s", exc)
            self.completed_status = "ERROR"
            self.state = "idle"
            self.comp_state = "ERROR"

        finally:
            if self.completed_status is not None:
                self.completed_command()

    def completed_command(self):
        pub_data = Completed()
        pub_data.command_id = self.comp_ref
        pub_data.status = self.completed_status
        self.pub.publish(pub_data)

        rospy.loginfo("Move completed: %s", self.completed_status)

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        rospy.loginfo("Move Component status: %s", self.comp_state)
        self.completed_status = None

    # ------------------------------------------------------------------
    # stop / suspend / resume
    # ------------------------------------------------------------------
    def _send_control_command(self, command):
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)

            request = MoveActionRequest()
            request.command = command
            request.line = []
            request.curve = []
            request.time = 0

            response = self.motion(request)

            if not response.success:
                rospy.logerr("Failed to %s Move: %s", command, response.status)
                return False

            return True

        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Move %s failed: %s", command, exc)
            return False

    def stop(self):
        if self.state not in ("moving", "suspended"):
            rospy.logwarn("No active Move operation.")
            return False

        if not self._send_control_command("stop"):
            return False

        self.state = "idle"
        return True

    def suspend(self):
        if self.state != "moving":
            rospy.logwarn("Cannot suspend Move; not moving.")
            return False

        if not self._send_control_command("suspend"):
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            rospy.logwarn("Cannot resume Move; not suspended.")
            return False

        if not self._send_control_command("resume"):
            return False

        self.state = "moving"
        return True

    # ------------------------------------------------------------------
    # Component status
    # ------------------------------------------------------------------
    def component_status(self, request):
        response = ComponentStatusResponse()
        if request.component_name == self.comp_ref:
            response.status = self.comp_state
        else:
            response.status = "ERROR"
        return response


def get_robotfile():
    package_path = rospkg.RosPack().get_path("common_hri_ros1")
    path = os.path.join(package_path, "robot.yaml")
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main():
    config = get_robotfile()
    robot = config["Robot"]
    robotname = "/" + robot

    rospy.init_node(robot + "_Move")
    MoveService(robotname)

    rospy.loginfo("ROS1 common Move started. robot=%s", robotname)
    rospy.spin()


if __name__ == "__main__":
    main()
