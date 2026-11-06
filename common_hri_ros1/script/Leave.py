#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS1 common Leave component.

MainEngine
  -> /<Robot>/execute/Leave (Execute action)
common_hri/Leave.py
  -> robot-specific RobotAction service
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
    RobotAction,
    RobotActionRequest,
)


class LeaveService:
    def __init__(self, robotname):
        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Leave"
        self.robotname = robotname

        self.state = "idle"
        self.completed_status = None
        self.motion_thread = None

        state_name = self.robotname + "/get_state/" + self.comp_ref
        self.state_service = rospy.Service(
            state_name,
            ComponentStatus,
            self.component_status,
        )
        rospy.loginfo("Leave component status server is ready: %s", state_name)

        exe_name = self.robotname + "/execute/" + self.comp_ref
        self.server = actionlib.SimpleActionServer(
            exe_name,
            ExecuteAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("Leave action server is ready: %s", exe_name)

        self.pub = rospy.Publisher(
            self.robotname + "/completed_command",
            Completed,
            queue_size=1,
        )

        config = get_robotfile()
        self.motion_service_name = config["Param"]["RobotAction"]["service"]
        self.motion = rospy.ServiceProxy(
            self.motion_service_name,
            RobotAction,
        )
        rospy.loginfo(
            "Leave RobotAction client configured: %s",
            self.motion_service_name,
        )

        self.comp_state = "READY"
        rospy.loginfo("Leave Component status: %s", self.comp_state)

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------
    def execute(self, goal):
        command = goal.command_name
        rospy.loginfo("Leave received command: %s", command)

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
            rospy.logwarn("Unknown Leave command: %s", command)
            ok = False

        result.success = "True" if ok else "False"

        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if self.comp_state != "READY":
            rospy.logwarn(
                "Cannot start Leave while component state is %s",
                self.comp_state,
            )
            return False

        self.state = "moving"
        self.completed_status = None
        self.comp_state = "BUSY"

        rospy.loginfo("Leave Component status: %s", self.comp_state)

        self.motion_thread = threading.Thread(
            target=self.monitor_leave,
            daemon=True,
        )
        self.motion_thread.start()
        return True

    def monitor_leave(self):
        rospy.loginfo("Monitoring Leave.")

        try:
            rospy.wait_for_service(self.motion_service_name, timeout=5.0)

            request = RobotActionRequest()
            #request.action_name = "leave"
            request.action_name = "test"  #通信テスト用

            service = self.motion(request)

            if service.success:
                self.completed_status = "OK"
                self.comp_state = "READY"
            else:
                self.completed_status = "ERROR"
                self.comp_state = "ERROR"

            self.state = "idle"

        except rospy.ROSException:
            rospy.logerr("RobotAction service wait timed out.")
            self.completed_status = "TIMEOUT"
            self.state = "idle"
            self.comp_state = "ERROR"

        except rospy.ServiceException as exc:
            rospy.logerr("RobotAction service call failed: %s", exc)
            self.completed_status = "ERROR"
            self.state = "idle"
            self.comp_state = "ERROR"

        except Exception as exc:
            rospy.logerr("Unexpected Leave error: %s", exc)
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

        rospy.loginfo("Leave completed: %s", self.completed_status)

        if self.completed_status in ("OK", "ABORT"):
            self.comp_state = "READY"
        else:
            self.comp_state = "ERROR"

        rospy.loginfo("Leave Component status: %s", self.comp_state)
        self.completed_status = None

    # Current common Leave has no generic lower-level stop/suspend/resume
    # interface. Keep the behavior explicit instead of pretending success.
    def stop(self):
        if self.state not in ("moving", "suspended"):
            rospy.logwarn("No active Leave operation.")
            return False

        rospy.logwarn("Leave stop is not supported by the current lower interface.")
        return False

    def suspend(self):
        if self.state != "moving":
            rospy.logwarn("Cannot suspend Leave; not moving.")
            return False

        rospy.logwarn("Leave suspend is not supported by the current lower interface.")
        return False

    def resume(self):
        if self.state != "suspended":
            rospy.logwarn("Cannot resume Leave; not suspended.")
            return False

        rospy.logwarn("Leave resume is not supported by the current lower interface.")
        return False

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

    rospy.init_node(robot + "_Leave")
    LeaveService(robotname)

    rospy.loginfo("ROS1 common Leave started. robot=%s", robotname)
    rospy.spin()


if __name__ == "__main__":
    main()
