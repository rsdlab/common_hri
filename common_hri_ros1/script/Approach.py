#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS1 common Approach component.

MainEngine
  -> /<Robot>/execute/Approach (Execute action)
  -> /<Robot>/approach_set_param, /approach_get_param
common_hri/Approach.py
  -> robot-specific ApproachAction service
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
    ApproachAction,
    ApproachActionRequest,
    ApproachGetParam,
    ApproachGetParamResponse,
    ApproachSetParam,
    ApproachSetParamResponse,
    ComponentStatus,
    ComponentStatusResponse,
)


class ApproachService:
    def __init__(self, robotname):
        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Approach"
        self.robotname = robotname

        self.state = "idle"  # idle / moving / suspended
        self.command = ""
        self.completed_status = None
        self.motion_thread = None

        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = rospy.Service(
            state_name,
            ComponentStatus,
            self.component_status,
        )
        rospy.loginfo("Approach component status server is ready: %s", state_name)

        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = actionlib.SimpleActionServer(
            exe_name,
            ExecuteAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("Approach action server is ready: %s", exe_name)

        self.pub = rospy.Publisher(
            self.robotname + "/completed_command",
            Completed,
            queue_size=1,
        )

        config = get_robotfile()
        self.motion_service_name = config["Param"]["ApproachAction"]["service"]
        self.motion = rospy.ServiceProxy(
            self.motion_service_name,
            ApproachAction,
        )
        rospy.loginfo(
            "ApproachAction client configured: %s",
            self.motion_service_name,
        )

        self.set_service = rospy.Service(
            self.robotname + "/approach_set_param",
            ApproachSetParam,
            self.set_parameter_callback,
        )
        self.get_service = rospy.Service(
            self.robotname + "/approach_get_param",
            ApproachGetParam,
            self.get_parameter_callback,
        )

        self.comp_state = "READY"
        rospy.loginfo("Approach Component status: %s", self.comp_state)

    # ------------------------------------------------------------------
    # Parameter
    # ------------------------------------------------------------------
    def set_parameter_callback(self, request):
        response = ApproachSetParamResponse()

        command = request.approach_command
        if not command:
            response.set_return = "BAD_PARAMETER"
            return response

        self.command = command
        response.set_return = "OK"

        rospy.loginfo("Approach parameter: %s", self.command)
        return response

    def get_parameter_callback(self, request):
        response = ApproachGetParamResponse()
        response.approach_command = self.command
        return response

    # ------------------------------------------------------------------
    # Execute action
    # ------------------------------------------------------------------
    def execute(self, goal):
        command = goal.command_name
        rospy.loginfo("Approach received command: %s", command)

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
            rospy.logwarn("Unknown Approach command: %s", command)
            ok = False

        result.success = "True" if ok else "False"

        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if self.comp_state != "READY":
            rospy.logwarn(
                "Cannot start Approach while component state is %s",
                self.comp_state,
            )
            return False

        if not self.command:
            rospy.logwarn("Approach parameter is not set.")
            return False

        self.state = "moving"
        self.comp_state = "BUSY"
        self.completed_status = None

        rospy.loginfo("Approach Component status: %s", self.comp_state)

        self.motion_thread = threading.Thread(
            target=self.monitor_playback,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_playback(self):
        rospy.loginfo("Monitoring Approach.")

        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)

            request = ApproachActionRequest()
            request.command = "start"
            request.action_name = self.command

            service = self.motion(request)

            self.completed_status = service.status
            self.state = "idle"

            if service.status in ("OK", "ABORT"):
                self.comp_state = "READY"
            else:
                self.comp_state = "ERROR"

        except rospy.ROSException:
            rospy.logerr("ApproachAction service wait timed out.")
            self.completed_status = "TIMEOUT"
            self.state = "idle"
            self.comp_state = "ERROR"

        except rospy.ServiceException as exc:
            rospy.logerr("ApproachAction service call failed: %s", exc)
            self.completed_status = "ERROR"
            self.state = "idle"
            self.comp_state = "ERROR"

        except Exception as exc:
            rospy.logerr("Unexpected Approach error: %s", exc)
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

        rospy.loginfo(
            "Approach completed: %s",
            self.completed_status,
        )

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        rospy.loginfo("Approach Component status: %s", self.comp_state)
        self.completed_status = None

    # ------------------------------------------------------------------
    # stop / suspend / resume
    # ------------------------------------------------------------------
    def _send_control_command(self, command):
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)

            request = ApproachActionRequest()
            request.command = command
            request.action_name = ""

            response = self.motion(request)

            if not response.success:
                rospy.logerr("Failed to %s Approach: %s", command, response.status)
                return False

            return True

        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Approach %s failed: %s", command, exc)
            return False

    def stop(self):
        if self.state not in ("moving", "suspended"):
            rospy.logwarn("No active Approach operation.")
            return False

        if not self._send_control_command("stop"):
            return False

        self.state = "idle"
        return True

    def suspend(self):
        if self.state != "moving":
            rospy.logwarn("Cannot suspend Approach; not moving.")
            return False

        if not self._send_control_command("suspend"):
            return False

        self.state = "suspended"
        return True

    def resume(self):
        if self.state != "suspended":
            rospy.logwarn("Cannot resume Approach; not suspended.")
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

    rospy.init_node(robot + "_Approach")
    ApproachService(robotname)

    rospy.loginfo("ROS1 common Approach started. robot=%s", robotname)
    rospy.spin()


if __name__ == "__main__":
    main()
