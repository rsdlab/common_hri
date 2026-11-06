#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 port of the revised ROS2 common Speech_Synthesis component.

MainEngine -> Execute action / speech_set_param -> common_hri_ros1
           -> robot-specific SpeechAction service.
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
    SpeechGetParam, SpeechGetParamResponse,
    SpeechSetParam, SpeechSetParamResponse,
    SpeechAction, SpeechActionRequest,
)


class SpeechSynthesisService:
    def __init__(self, robotname):
        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Speech_Synthesis"
        self.robotname = robotname
        self.text = ""
        self.state = "idle"
        self.playback_thread = None

        self.state_service = rospy.Service(
            self.robotname + "/get_state/" + self.comp_ref,
            ComponentStatus, self.component_status,
        )
        self.server = actionlib.SimpleActionServer(
            self.robotname + "/execute/" + self.comp_ref,
            ExecuteAction, execute_cb=self.execute, auto_start=False,
        )
        self.set_service = rospy.Service(
            self.robotname + "/speech_set_param",
            SpeechSetParam, self.set_parameter_callback,
        )
        self.get_service = rospy.Service(
            self.robotname + "/speech_get_param",
            SpeechGetParam, self.get_parameter_callback,
        )
        self.pub = rospy.Publisher(
            self.robotname + "/completed_command", Completed, queue_size=1,
        )

        config = get_robotfile()
        self.speech_service_name = config["Param"]["SpeechSynthesisAction"]["service"]
        self.speech_client = rospy.ServiceProxy(
            self.speech_service_name, SpeechAction,
        )
        self.comp_state = "READY"
        self.server.start()
        rospy.loginfo("Speech_Synthesis ready. implementation=%s", self.speech_service_name)

    def set_parameter_callback(self, request):
        response = SpeechSetParamResponse()
        speech_text = request.speech_text.strip()
        if not speech_text:
            response.set_return = "BAD_PARAMETER"
            rospy.logwarn("speech_text is empty.")
            return response
        self.text = speech_text
        response.set_return = "OK"
        rospy.loginfo("Speech text set: %s", self.text)
        return response

    def get_parameter_callback(self, request):
        response = SpeechGetParamResponse()
        response.speech_text = self.text
        return response

    def execute(self, goal):
        self.comp_state = "BUSY"
        command = goal.command_name
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
            rospy.logwarn("Unsupported command: %s", command)
            self.comp_state = "READY"
            ok = False
        result.success = "True" if ok else "False"
        if ok:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def start(self):
        if not self.text:
            rospy.logwarn("Speech Synthesis parameter is not set.")
            self.comp_state = "READY"
            return False
        if self.playback_thread and self.playback_thread.is_alive():
            rospy.logwarn("Speech Synthesis is already running.")
            self.comp_state = "BUSY"
            return False
        self.state = "playing"
        self.playback_thread = threading.Thread(
            target=self.monitor_playback, daemon=True,
        )
        self.playback_thread.start()
        return True

    def monitor_playback(self):
        try:
            rospy.wait_for_service(self.speech_service_name, timeout=5.0)
            request = SpeechActionRequest()
            # ROS1 SpeechAction.srv has an additional 'command' field.
            # The revised ROS2 component passes only speech_text; its lower
            # operation is a one-shot speech synthesis request.
            request.command = "start"
            request.speech_text = self.text
            response = self.speech_client(request)
            if response.success:
                self.state = "OK"
            else:
                self.state = "ERROR"
                self.comp_state = "ERROR"
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Speech Synthesis failed: %s", exc)
            self.state = "ERROR"
            self.comp_state = "ERROR"
        except Exception as exc:
            rospy.logerr("Speech Synthesis failed: %s", exc)
            self.state = "ERROR"
            self.comp_state = "ERROR"
        finally:
            self.completed_command()

    def completed_command(self):
        pub_data = Completed()
        pub_data.command_id = self.comp_ref
        pub_data.status = self.state
        self.pub.publish(pub_data)
        if self.comp_state != "ERROR":
            self.comp_state = "READY"
        rospy.loginfo("Speech Synthesis completed: %s", self.state)

    def stop(self):
        rospy.logwarn("stop is not supported by the current SpeechAction interface.")
        self.comp_state = "READY" if self.state != "playing" else "BUSY"
        return False

    def suspend(self):
        rospy.logwarn("suspend is not supported by the current SpeechAction interface.")
        self.comp_state = "READY" if self.state != "playing" else "BUSY"
        return False

    def resume(self):
        rospy.logwarn("resume is not supported by the current SpeechAction interface.")
        self.comp_state = "READY" if self.state != "playing" else "BUSY"
        return False

    def component_status(self, request):
        response = ComponentStatusResponse()
        response.status = (
            self.comp_state if request.component_name == self.comp_ref
            else "UNKNOWN_COMPONENT"
        )
        return response


def get_robotfile():
    package_path = rospkg.RosPack().get_path("common_hri_ros1")
    with open(os.path.join(package_path, "robot.yaml"), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    robot = get_robotfile()["Robot"]
    rospy.init_node(robot + "_Speech_Synthesis")
    SpeechSynthesisService("/" + robot)
    rospy.spin()


if __name__ == "__main__":
    main()
