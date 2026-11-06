#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 port of the revised ROS2 common Speech_Recognition component.

MainEngine -> Execute action / s_recognition_set_param -> common_hri_ros1
           -> robot-specific SpeechRecognitionAction service
           -> NotifySpeechrec event (not Completed).
"""
import os
import threading
from datetime import datetime

import actionlib
import rospy
import rospkg
import yaml

from rois_env_ros1.msg import ExecuteAction, ExecuteResult, NotifySpeechrec
from rois_env_ros1.srv import (
    ComponentStatus, ComponentStatusResponse,
    SpeechRecognitionGetParam, SpeechRecognitionGetParamResponse,
    SpeechRecognitionSetParam, SpeechRecognitionSetParamResponse,
    SpeechRecognitionAction, SpeechRecognitionActionRequest,
)


class SpeechRecognitionService:
    def __init__(self, robotname):
        self.comp_state = "UNINITIALIZED"
        self.comp_ref = "Speech_Recognition"
        self.robotname = robotname
        self.recognizable_list = ["japanese", "english", "french"]
        self.language_map = {
            "japanese": "ja-JP", "english": "en-US", "french": "fr-FR",
        }
        self.set_language = "japanese"
        self.languages = "ja-JP"
        self.recognized_text = ""
        self.recognition_thread = None
        self.state = "idle"

        self.state_service = rospy.Service(
            self.robotname + "/get_state/" + self.comp_ref,
            ComponentStatus, self.component_status,
        )
        self.server = actionlib.SimpleActionServer(
            self.robotname + "/execute/" + self.comp_ref,
            ExecuteAction, execute_cb=self.execute, auto_start=False,
        )
        self.set_service = rospy.Service(
            self.robotname + "/s_recognition_set_param",
            SpeechRecognitionSetParam, self.set_parameter_callback,
        )
        self.get_service = rospy.Service(
            self.robotname + "/s_recognition_get_param",
            SpeechRecognitionGetParam, self.get_parameter_callback,
        )
        self.pub = rospy.Publisher(
            self.robotname + "/event_speechrec", NotifySpeechrec, queue_size=1,
        )
        config = get_robotfile()
        self.recognition_service_name = config["Param"]["SpeechRecognitionAction"]["service"]
        self.recognition_client = rospy.ServiceProxy(
            self.recognition_service_name, SpeechRecognitionAction,
        )
        self.comp_state = "READY"
        self.server.start()
        rospy.loginfo("Speech_Recognition ready. implementation=%s", self.recognition_service_name)

    def set_parameter_callback(self, request):
        response = SpeechRecognitionSetParamResponse()
        language = request.languages.strip().lower()
        if language not in self.recognizable_list:
            rospy.logwarn("Unsupported language: %s", language)
            response.set_return = "BAD_PARAMETER"
            return response
        self.set_language = language
        self.languages = self.language_map[language]
        response.set_return = "OK"
        rospy.loginfo("Speech recognition language set: %s (%s)",
                      self.set_language, self.languages)
        return response

    def get_parameter_callback(self, request):
        response = SpeechRecognitionGetParamResponse()
        # ROS1 response also requires the supported-language list.
        response.recognizable_languages = list(self.recognizable_list)
        response.languages = self.set_language
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
        if self.recognition_thread and self.recognition_thread.is_alive():
            rospy.logwarn("Speech recognition is already running.")
            self.comp_state = "BUSY"
            return False
        self.state = "recognizing"
        self.recognition_thread = threading.Thread(
            target=self.recognize, daemon=True,
        )
        self.recognition_thread.start()
        return True

    def recognize(self):
        try:
            rospy.wait_for_service(self.recognition_service_name, timeout=5.0)
            request = SpeechRecognitionActionRequest()
            request.language = self.languages
            rospy.loginfo("Start speech recognition: %s", self.languages)
            response = self.recognition_client(request)
            if not response.success:
                raise RuntimeError("Speech recognition failed.")
            self.recognized_text = response.recognized_text
            rospy.loginfo("Recognized text: %s", self.recognized_text)
            self.publish_recognition_result(self.recognized_text)
            self.state = "OK"
            self.comp_state = "READY"
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Speech Recognition failed: %s", exc)
            self.state = "ERROR"
            self.comp_state = "ERROR"
        except Exception as exc:
            rospy.logerr("Speech Recognition failed: %s", exc)
            self.state = "ERROR"
            self.comp_state = "ERROR"

    def publish_recognition_result(self, text):
        msg = NotifySpeechrec()
        msg.event_type = "speech_recognized"
        msg.timestamp = datetime.now().isoformat()
        msg.recognized_text = text
        self.pub.publish(msg)

    def stop(self):
        rospy.logwarn("stop is not supported by the current SpeechRecognitionAction interface.")
        self.comp_state = "BUSY" if self.state == "recognizing" else "READY"
        return False

    def suspend(self):
        rospy.logwarn("suspend is not supported by the current SpeechRecognitionAction interface.")
        self.comp_state = "BUSY" if self.state == "recognizing" else "READY"
        return False

    def resume(self):
        rospy.logwarn("resume is not supported by the current SpeechRecognitionAction interface.")
        self.comp_state = "BUSY" if self.state == "recognizing" else "READY"
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
    rospy.init_node(robot + "_Speech_Recognition")
    SpeechRecognitionService("/" + robot)
    rospy.spin()


if __name__ == "__main__":
    main()
