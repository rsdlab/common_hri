#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 RoIS Object_Release common component.

AIREC has no lower ObjectReleaseAction provider yet; unavailable service
is reported as TIMEOUT, not a successful robot operation.
"""
import os
import threading

import actionlib
import rospy
import rospkg
import yaml

from rois_env_ros1.msg import Completed, ExecuteAction, ExecuteResult
from rois_env_ros1.srv import (
    ComponentStatus, ComponentStatusResponse, RobotAction, RobotActionRequest,
)


class ObjectReleaseService:
    def __init__(self, robotname):
        self.comp_ref = "Object_Release"
        self.robotname = robotname
        self.comp_state = "UNINITIALIZED"
        self.state = "idle"
        self.completed_status = None
        self.motion_thread = None

        self.state_service = rospy.Service(
            robotname + "/get_state/" + self.comp_ref,
            ComponentStatus, self.component_status,
        )
        self.server = actionlib.SimpleActionServer(
            robotname + "/execute/" + self.comp_ref,
            ExecuteAction, execute_cb=self.execute, auto_start=False,
        )
        self.server.start()
        self.pub = rospy.Publisher(
            robotname + "/completed_command", Completed, queue_size=1,
        )
        config = get_robotfile()
        self.motion_service_name = config["Param"]["ObjectReleaseAction"]["service"]
        self.motion = rospy.ServiceProxy(self.motion_service_name, RobotAction)
        self.comp_state = "READY"
        rospy.loginfo("Object Release READY (lower service: %s)", self.motion_service_name)

    def execute(self, goal):
        command = goal.command_name
        rospy.loginfo("Object Release command: %s", command)
        if command == "start":
            ok = self.start()
        elif command in ("stop", "suspend", "resume"):
            ok = self._send_control_command(command)
        else:
            rospy.logwarn("Unknown Object Release command: %s", command)
            ok = False
        result = ExecuteResult()
        result.success = "True" if ok else "False"
        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if self.comp_state != "READY":
            rospy.logwarn("Cannot start Object Release: state=%s", self.comp_state)
            return False
        self.state = "releasing"
        self.comp_state = "BUSY"
        self.completed_status = None
        self.motion_thread = threading.Thread(target=self.monitor_release, daemon=True)
        self.motion_thread.start()
        return True

    def monitor_release(self):
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)
            request = RobotActionRequest()
            request.action_name = "start"
            response = self.motion(request)
            self.completed_status = response.status if response.success else "ERROR"
            if self.completed_status not in ("OK", "ABORT"):
                self.completed_status = "ERROR"
        except rospy.ROSException:
            rospy.logwarn("ObjectReleaseAction service not available: %s",
                          self.motion_service_name)
            self.completed_status = "TIMEOUT"
        except (rospy.ServiceException, Exception) as exc:
            rospy.logerr("Object Release failed: %s", exc)
            self.completed_status = "ERROR"
        finally:
            self.state = "idle"
            self.completed_command()

    def completed_command(self):
        message = Completed()
        message.command_id = self.comp_ref
        message.status = self.completed_status or "ERROR"
        self.pub.publish(message)
        self.comp_state = "READY" if message.status in ("OK", "ABORT") else "ERROR"
        rospy.loginfo("Object Release completed: %s", message.status)
        self.completed_status = None

    def _send_control_command(self, command):
        if command == "stop" and self.state not in ("releasing", "suspended"):
            return False
        if command == "suspend" and self.state != "releasing":
            return False
        if command == "resume" and self.state != "suspended":
            return False
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)
            request = RobotActionRequest()
            request.action_name = command
            response = self.motion(request)
            if not response.success:
                return False
            if command == "stop":
                self.state = "idle"
            elif command == "suspend":
                self.state = "suspended"
            elif command == "resume":
                self.state = "releasing"
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Object Release %s failed: %s", command, exc)
            return False

    def component_status(self, request):
        response = ComponentStatusResponse()
        response.status = self.comp_state if request.component_name == self.comp_ref else "ERROR"
        return response


def get_robotfile():
    path = os.path.join(rospkg.RosPack().get_path("common_hri_ros1"), "robot.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    robot = get_robotfile()["Robot"]
    rospy.init_node(robot + "_Object_Release")
    ObjectReleaseService("/" + robot)
    rospy.spin()


if __name__ == "__main__":
    main()
