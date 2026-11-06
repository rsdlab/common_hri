#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 RoIS Object_Grasping common component.

AIREC currently has no lower ObjectGraspingAction provider. The component can
start independently; requesting execution without a provider reports TIMEOUT.
"""
import os
import threading

import actionlib
import rospy
import rospkg
import yaml

from rois_env_ros1.msg import Completed, ExecuteAction, ExecuteResult
from rois_env_ros1.srv import (
    ComponentStatus, ComponentStatusResponse,
    ObjectGraspingAction, ObjectGraspingActionRequest,
    ObjectGraspingGetParam, ObjectGraspingGetParamResponse,
    ObjectGraspingSetParam, ObjectGraspingSetParamResponse,
)


class ObjectGraspingService:
    def __init__(self, robotname):
        self.comp_ref = "Object_Grasping"
        self.robotname = robotname
        self.comp_state = "UNINITIALIZED"
        self.state = "idle"
        self.completed_status = None
        self.motion_thread = None
        self.object_id = ""
        self.grasp_force = 0.0

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
        self.motion_service_name = config["Param"]["ObjectGraspingAction"]["service"]
        self.motion = rospy.ServiceProxy(
            self.motion_service_name, ObjectGraspingAction,
        )
        self.set_service = rospy.Service(
            robotname + "/object_grasping_set_param",
            ObjectGraspingSetParam, self.set_parameter_callback,
        )
        self.get_service = rospy.Service(
            robotname + "/object_grasping_get_param",
            ObjectGraspingGetParam, self.get_parameter_callback,
        )
        self.comp_state = "READY"
        rospy.loginfo("Object Grasping READY (lower service: %s)", self.motion_service_name)

    def set_parameter_callback(self, request):
        response = ObjectGraspingSetParamResponse()
        object_id = request.object_id.strip()
        grasp_force = float(request.grasp_force)
        if not object_id or grasp_force < 0.0:
            response.set_return = "BAD_PARAMETER"
            return response
        self.object_id = object_id
        self.grasp_force = grasp_force
        rospy.loginfo("Grasp parameters: object_id=%s, grasp_force=%s N",
                      self.object_id, self.grasp_force)
        response.set_return = "OK"
        return response

    def get_parameter_callback(self, request):
        response = ObjectGraspingGetParamResponse()
        response.object_id = self.object_id
        response.grasp_force = self.grasp_force
        return response

    def execute(self, goal):
        command = goal.command_name
        rospy.loginfo("Object Grasping command: %s", command)
        if command == "start":
            ok = self.start()
        elif command in ("stop", "suspend", "resume"):
            ok = self._send_control_command(command)
        else:
            rospy.logwarn("Unknown Object Grasping command: %s", command)
            ok = False
        result = ExecuteResult()
        result.success = "True" if ok else "False"
        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if self.comp_state != "READY" or not self.object_id:
            rospy.logwarn("Grasp cannot start: state=%s, object_id=%r",
                          self.comp_state, self.object_id)
            return False
        self.state = "grasping"
        self.comp_state = "BUSY"
        self.completed_status = None
        self.motion_thread = threading.Thread(target=self.monitor_grasping, daemon=True)
        self.motion_thread.start()
        return True

    def monitor_grasping(self):
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)
            request = ObjectGraspingActionRequest()
            request.command = "start"
            request.object_id = self.object_id
            request.grasp_force = self.grasp_force
            response = self.motion(request)
            self.completed_status = response.status if response.success else "ERROR"
            if self.completed_status not in ("OK", "ABORT"):
                self.completed_status = "ERROR"
        except rospy.ROSException:
            rospy.logwarn("ObjectGraspingAction service not available: %s",
                          self.motion_service_name)
            self.completed_status = "TIMEOUT"
        except (rospy.ServiceException, Exception) as exc:
            rospy.logerr("Object Grasping failed: %s", exc)
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
        rospy.loginfo("Object Grasping completed: %s", message.status)
        self.completed_status = None

    def _send_control_command(self, command):
        if command == "stop" and self.state not in ("grasping", "suspended"):
            return False
        if command == "suspend" and self.state != "grasping":
            return False
        if command == "resume" and self.state != "suspended":
            return False
        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)
            request = ObjectGraspingActionRequest()
            request.command = command
            request.object_id = ""
            request.grasp_force = 0.0
            response = self.motion(request)
            if not response.success:
                return False
            if command == "suspend":
                self.state = "suspended"
            elif command == "resume":
                self.state = "grasping"
            elif command == "stop":
                self.state = "idle"
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Object Grasping %s failed: %s", command, exc)
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
    rospy.init_node(robot + "_Object_Grasping")
    ObjectGraspingService("/" + robot)
    rospy.spin()


if __name__ == "__main__":
    main()
