#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Unified ROS1 MainEngine for AIREC / SEED / HSR.

Policy:
- MainEngine is robot-independent.
- Robot differences are expressed by hri.xml / robot.yaml.
- MainEngine talks only to common_hri interfaces.
- Robot-specific APIs belong below common_hri in *_hri_action packages.

ROS1 counterpart of Mainengine_ros2.py.
"""

import os
import xml.etree.ElementTree as ET

import actionlib
import rospy
import rospkg
import yaml
from actionlib_msgs.msg import GoalStatus

from rois_env_ros1.srv import *
from rois_env_ros1.msg import *


class EngineService:
    def __init__(self, robotname):
        self.robotname = robotname
        self.ENGINE_RECEIVABLE = False
        self.BINDCOMP = None
        self.component_ref = None

        # Engine state / cached parameters
        self.command_list = []
        self.event_list = []
        self.speech_recognition_list = []
        self.person_detected_list = []
        self.person_identi_list = []
        self.person_position_list = []

        self.last_timestamp = ""
        self.event_count = 0
        self.command_count = 0
        self.subcount = 0
        self.subscribe_id = ""

        self.text = "hello"
        self.language = "japanese"
        self.recog_language = ["japanese"]
        self.line = [500, 0, 0]
        self.curve = []
        self.move_time = 0
        self.command = ""
        self.pick_and_place_command = ""
        self.shelf_command = ""
        self.target_position = []
        self.object_id = ""
        self.grasp_force = 0.0

        # Dynamically created ROS entities
        self.action_client = None
        self.parameter_get_service = None
        self.parameter_set_service = None

        self.event_subscriber1 = None
        self.event_subscriber2 = None
        self.event_subscriber3 = None
        self.event_subscriber4 = None

        # ------------------------------------------------------------
        # Service Application <-> MainEngine
        # ------------------------------------------------------------
        self.con = rospy.Service("/connect", SystemInterface, self.connect)
        self.discon = rospy.Service("/disconnect", SystemInterface, self.disconnect)
        self.bind = rospy.Service("/bind_any", BindAny, self.bind_any)
        self.release_srv = rospy.Service("/release", Release, self.release)
        self.execute_srv = rospy.Service("/execute", Execute, self.execute_cb)
        self.subscribe_srv = rospy.Service("/subscribe", Subscribe, self.subscribe_cb)
        self.unsubscribe_srv = rospy.Service(
            "/unsubscribe", Unsubscribe, self.unsubscribe_cb
        )

        self.eventdetailservice1 = rospy.Service(
            "/recognized_event_detail",
            GetEventDetailSpeechRecognized,
            self.event_recognized_cb,
        )
        self.eventdetailservice2 = rospy.Service(
            "/detected_event_detail",
            GetEventDetailPersonDetected,
            self.event_detected_cb,
        )
        self.eventdetailservice3 = rospy.Service(
            "/local_event_detail",
            GetEventDetailPersonLocalized,
            self.event_local_cb,
        )
        self.eventdetailservice4 = rospy.Service(
            "/identified_event_detail",
            GetEventDetailPersonIdentified,
            self.event_identified_cb,
        )

        self.get_command_result_service = rospy.Service(
            "/get_command_result",
            GetCommandResult,
            self.get_command_result_cb,
        )

        # Component -> Engine -> App
        self.completed_sub = rospy.Subscriber(
            self.robotname + "/completed_command",
            Completed,
            self.completed,
            queue_size=10,
        )
        self.completed_pub = rospy.Publisher(
            "/completed", Completed, queue_size=10
        )
        self.notify_event_pub = rospy.Publisher(
            "/notify_event", Notifyevent, queue_size=10
        )

        rospy.loginfo("Engine Service is ready. robot=%s", self.robotname)

    # ================================================================
    # Utilities
    # ================================================================
    @staticmethod
    def _set_if(obj, field, value):
        if hasattr(obj, field):
            setattr(obj, field, value)

    @staticmethod
    def _response_code(response, value):
        # Historical interfaces use multiple names.
        for field in ("returncode_t", "return_t", "set_return", "result"):
            if hasattr(response, field):
                setattr(response, field, value)
                return

    @staticmethod
    def _new_response(srv_type):
        return srv_type._response_class()

    @staticmethod
    def _new_request(srv_type):
        return srv_type._request_class()

    def _call_service(self, srv_type, service_name, request=None, timeout_sec=5.0):
        rospy.wait_for_service(service_name, timeout=timeout_sec)
        client = rospy.ServiceProxy(service_name, srv_type)
        if request is None:
            request = self._new_request(srv_type)
        return client(request)

    def is_service_available(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
            return True
        except rospy.ROSException:
            return False

    def get_component(self):
        """Read available HRIComponent entries from common_hri/hri.xml."""
        package_path = rospkg.RosPack().get_path("common_hri_ros1")
        file_path = os.path.join(package_path, "hri.xml")

        tree = ET.parse(file_path)
        root = tree.getroot()

        namespaces = {
            "rois": "http://example.com/r/rois",
            "gml": "http://example.com/r/gml",
        }

        components = []
        for elem in root.findall("rois:HRIComponent", namespaces):
            if elem.text is None:
                continue
            text = elem.text.strip()

            if "::" in text:
                name = text.split("::")[-1]
            else:
                name = text.replace("rois:HRIComponent:", "")

            components.append(name)

        return components

    def _destroy_dynamic_parameter_services(self):
        if self.parameter_get_service is not None:
            try:
                self.parameter_get_service.shutdown("release")
            except Exception:
                pass
            self.parameter_get_service = None

        if self.parameter_set_service is not None:
            try:
                self.parameter_set_service.shutdown("release")
            except Exception:
                pass
            self.parameter_set_service = None

    def _create_action_client(self, component_ref):
        exe_name = self.robotname + "/execute/" + component_ref

        self.action_client = actionlib.SimpleActionClient(
            exe_name,
            ExecuteAction,
        )

        rospy.loginfo("Waiting for action server: %s", exe_name)
        if not self.action_client.wait_for_server(rospy.Duration(5.0)):
            self.action_client = None
            raise RuntimeError("Action server not available: " + exe_name)

        rospy.loginfo("Action client ready: %s", exe_name)

    # ================================================================
    # System Interface
    # ================================================================
    def connect(self, request):
        response = self._new_response(SystemInterface)

        if request.connect == "connect":
            self.ENGINE_RECEIVABLE = True
            rospy.loginfo("Engine is receivable")
            self._response_code(response, "OK")
        else:
            self._response_code(response, "ERROR")

        return response

    def disconnect(self, request):
        response = self._new_response(SystemInterface)

        if request.connect == "disconnect":
            self.ENGINE_RECEIVABLE = False
            rospy.loginfo("Engine is not receivable")
            self._response_code(response, "OK")
        else:
            self._response_code(response, "ERROR")

        return response

    # ================================================================
    # Command Interface: bind / release
    # ================================================================
    def bind_any(self, request):
        response = self._new_response(BindAny)

        try:
            self.component_ref_list = self.get_component()
            self.component_ref = request.component_ref

            rospy.loginfo(
                "bind_any: requested=%s, available=%s",
                self.component_ref,
                self.component_ref_list,
            )

            if not self.ENGINE_RECEIVABLE:
                self._response_code(response, "ERROR")
                return response

            if self.BINDCOMP is not None:
                rospy.logwarn("Already bound: %s", self.BINDCOMP)
                self._response_code(response, "ERROR")
                return response

            if self.component_ref not in self.component_ref_list:
                rospy.logwarn(
                    "HRI-Component does not exist: %s",
                    self.component_ref,
                )
                self._response_code(response, "ERROR")
                return response

            get_name = "/get_parameter/" + self.component_ref
            set_name = "/set_parameter/" + self.component_ref

            self._destroy_dynamic_parameter_services()

            # --------------------------------------------------------
            # Components with Engine-level parameter bridge
            # --------------------------------------------------------
            if self.component_ref == "Speech_Synthesis":
                self.text = "hello"
                self.parameter_get_service = rospy.Service(
                    get_name,
                    SpeechGetParameter,
                    self.get_speech,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    SpeechSetParameter,
                    self.set_speech,
                )

            elif self.component_ref == "Speech_Recognition":
                self.language = "japanese"
                self.parameter_get_service = rospy.Service(
                    get_name,
                    SpeechRecognitionGetParameter,
                    self.get_s_recog,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    SpeechRecognitionSetParameter,
                    self.set_s_recog,
                )

            elif self.component_ref == "Move":
                self.line = [500, 0, 0]
                self.curve = []
                self.move_time = 0
                self.parameter_get_service = rospy.Service(
                    get_name,
                    MoveGetParameter,
                    self.get_move,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    MoveSetParameter,
                    self.set_move,
                )

            elif self.component_ref == "Approach":
                self.command = "approach1"
                self.parameter_get_service = rospy.Service(
                    get_name,
                    ApproachGetParameter,
                    self.get_approach,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    ApproachSetParameter,
                    self.set_approach,
                )

            # Mainengine2/3/4 compatibility.
            # They use the same command-shaped parameter interface as Approach.
            elif self.component_ref == "Pick_and_Place":
                self.pick_and_place_command = ""
                self.parameter_get_service = rospy.Service(
                    get_name,
                    ApproachGetParameter,
                    self.get_pick_and_place,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    ApproachSetParameter,
                    self.set_pick_and_place,
                )

            elif self.component_ref == "Shelf":
                self.shelf_command = ""
                self.parameter_get_service = rospy.Service(
                    get_name,
                    ApproachGetParameter,
                    self.get_shelf,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    ApproachSetParameter,
                    self.set_shelf,
                )

            elif self.component_ref == "Navigation":
                self.target_position = []
                self.parameter_get_service = rospy.Service(
                    get_name,
                    NaviGetParameter,
                    self.get_navi,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    NaviSetParameter,
                    self.set_navi,
                )

            elif self.component_ref == "Object_Grasping":
                self.object_id = ""
                self.grasp_force = 0.0
                self.parameter_get_service = rospy.Service(
                    get_name,
                    ObjectGraspingGetParameter,
                    self.get_object_grasping,
                )
                self.parameter_set_service = rospy.Service(
                    set_name,
                    ObjectGraspingSetParameter,
                    self.set_object_grasping,
                )

            # Action-only components.
            elif self.component_ref in {
                "Person_Detection",
                "Person_Localization",
                "Person_Identification",
                "Touch",
                "Touch_Detection",
                "Leave",
                "Pick",
                "Place",
                "Object_Release",
            }:
                pass

            else:
                rospy.logwarn(
                    "No bind rule implemented for %s",
                    self.component_ref,
                )
                self._response_code(response, "ERROR")
                return response

            self._create_action_client(self.component_ref)

            self.BINDCOMP = self.component_ref
            rospy.loginfo("Bind %s", self.component_ref)
            self._response_code(response, "OK")

        except Exception as exc:
            rospy.logerr("bind_any failed: %s", exc)
            self._destroy_dynamic_parameter_services()
            self.action_client = None
            self.BINDCOMP = None
            self.component_ref = None
            self._response_code(response, "ERROR")

        return response

    def release(self, request):
        response = self._new_response(Release)

        try:
            self._destroy_dynamic_parameter_services()

            if self.action_client is not None:
                try:
                    self.action_client.cancel_all_goals()
                except Exception:
                    pass
                self.action_client = None

            self.BINDCOMP = None
            self.component_ref = None

            self._response_code(response, "OK")

        except Exception as exc:
            rospy.logerr("release failed: %s", exc)
            self._response_code(response, "ERROR")

        return response

    # ================================================================
    # Parameter bridge: Speech Synthesis
    # ================================================================
    def set_speech(self, request):
        response = self._new_response(SpeechSetParameter)
        command_id = "set_speech_synthesis"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.text = request.speech_text

            lower_req = self._new_request(SpeechSetParam)
            self._set_if(lower_req, "speech_text", self.text)

            result = self._call_service(
                SpeechSetParam,
                self.robotname + "/speech_set_param",
                lower_req,
            )

            code = getattr(result, "set_return", "ERROR")
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)

        except Exception as exc:
            rospy.logerr("set_speech failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_speech(self, request):
        response = self._new_response(SpeechGetParameter)

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "speech_text", "")
            return response

        try:
            result = self._call_service(
                SpeechGetParam,
                self.robotname + "/speech_get_param",
            )

            self.text = result.speech_text
            self._response_code(response, "OK")
            self._set_if(response, "speech_text", self.text)

        except Exception as exc:
            rospy.logerr("get_speech failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "speech_text", "")

        return response

    # ================================================================
    # Parameter bridge: Speech Recognition
    # ================================================================
    def set_s_recog(self, request):
        response = self._new_response(SpeechRecognitionSetParameter)
        command_id = "set_speech_recognition"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.language = request.languages

            lower_req = self._new_request(SpeechRecognitionSetParam)
            self._set_if(lower_req, "languages", self.language)

            result = self._call_service(
                SpeechRecognitionSetParam,
                self.robotname + "/s_recognition_set_param",
                lower_req,
            )

            code = getattr(result, "set_return", "ERROR")
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)

        except Exception as exc:
            rospy.logerr("set_s_recog failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_s_recog(self, request):
        response = self._new_response(SpeechRecognitionGetParameter)

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "recognizable_languages", [])
            self._set_if(response, "languages", "")
            return response

        try:
            result = self._call_service(
                SpeechRecognitionGetParam,
                self.robotname + "/s_recognition_get_param",
            )

            self.language = result.languages
            self.recog_language = list(result.recognizable_languages)

            self._response_code(response, "OK")
            self._set_if(response, "recognizable_languages", self.recog_language)
            self._set_if(response, "languages", self.language)

        except Exception as exc:
            rospy.logerr("get_s_recog failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "recognizable_languages", [])
            self._set_if(response, "languages", "")

        return response

    # ================================================================
    # Parameter bridge: Move
    # ================================================================
    def set_move(self, request):
        response = self._new_response(MoveSetParameter)
        command_id = "set_move"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.line = list(request.line)
            self.curve = list(request.curve)
            self.move_time = getattr(request, "time", 0)

            lower_req = self._new_request(MoveSetParam)
            self._set_if(lower_req, "line", self.line)
            self._set_if(lower_req, "curve", self.curve)
            self._set_if(lower_req, "time", self.move_time)

            result = self._call_service(
                MoveSetParam,
                self.robotname + "/move_set_param",
                lower_req,
            )

            code = getattr(
                result,
                "returncode_t",
                getattr(result, "set_return", "ERROR"),
            )
            self._response_code(response, code)
            self._set_if(
                response,
                "command_id",
                getattr(result, "command_id", command_id),
            )

        except Exception as exc:
            rospy.logerr("set_move failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_move(self, request):
        response = self._new_response(MoveGetParameter)

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "line", [])
            self._set_if(response, "curve", [])
            return response

        try:
            result = self._call_service(
                MoveGetParam,
                self.robotname + "/move_get_param",
            )

            self.line = list(result.line)
            self.curve = list(result.curve)
            self.move_time = getattr(result, "time", 0)

            self._response_code(
                response,
                getattr(result, "returncode_t", "OK"),
            )
            self._set_if(response, "line", self.line)
            self._set_if(response, "curve", self.curve)
            self._set_if(response, "time", self.move_time)

        except Exception as exc:
            rospy.logerr("get_move failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "line", [])
            self._set_if(response, "curve", [])

        return response

    # ================================================================
    # Parameter bridge: Approach / Pick_and_Place / Shelf
    # ================================================================
    def _set_approach_like(
        self,
        value,
        lower_service_name,
        command_id,
        response,
    ):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            lower_req = self._new_request(ApproachSetParam)
            self._set_if(lower_req, "approach_command", value)

            result = self._call_service(
                ApproachSetParam,
                lower_service_name,
                lower_req,
            )

            code = getattr(
                result,
                "set_return",
                getattr(result, "returncode_t", "ERROR"),
            )
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)

        except Exception as exc:
            rospy.logerr("%s failed: %s", command_id, exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def _get_approach_like(self, lower_service_name, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "approach_command", "")
            return response, ""

        try:
            result = self._call_service(
                ApproachGetParam,
                lower_service_name,
            )

            value = result.approach_command
            self._response_code(response, "OK")
            self._set_if(response, "approach_command", value)
            return response, value

        except Exception as exc:
            rospy.logerr("get parameter failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "approach_command", "")
            return response, ""

    def set_approach(self, request):
        response = self._new_response(ApproachSetParameter)
        self.command = request.approach_command
        return self._set_approach_like(
            self.command,
            self.robotname + "/approach_set_param",
            "set_approach",
            response,
        )

    def get_approach(self, request):
        response = self._new_response(ApproachGetParameter)
        response, self.command = self._get_approach_like(
            self.robotname + "/approach_get_param",
            response,
        )
        return response

    def set_pick_and_place(self, request):
        response = self._new_response(ApproachSetParameter)
        self.pick_and_place_command = request.approach_command
        return self._set_approach_like(
            self.pick_and_place_command,
            self.robotname + "/pick_and_place_set_param",
            "set_pick_and_place",
            response,
        )

    def get_pick_and_place(self, request):
        response = self._new_response(ApproachGetParameter)
        response, self.pick_and_place_command = self._get_approach_like(
            self.robotname + "/pick_and_place_get_param",
            response,
        )
        return response

    def set_shelf(self, request):
        response = self._new_response(ApproachSetParameter)
        self.shelf_command = request.approach_command
        return self._set_approach_like(
            self.shelf_command,
            self.robotname + "/shelf_set_param",
            "set_shelf",
            response,
        )

    def get_shelf(self, request):
        response = self._new_response(ApproachGetParameter)
        response, self.shelf_command = self._get_approach_like(
            self.robotname + "/shelf_get_param",
            response,
        )
        return response

    # ================================================================
    # Parameter bridge: Navigation
    # ================================================================
    def set_navi(self, request):
        response = self._new_response(NaviSetParameter)
        command_id = "set_navigation"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.target_position = list(request.target_position)

            lower_req = self._new_request(NaviSetParam)
            self._set_if(lower_req, "target_position", self.target_position)

            result = self._call_service(
                NaviSetParam,
                self.robotname + "/navi_set_param",
                lower_req,
            )

            self._response_code(
                response,
                getattr(result, "returncode_t", "ERROR"),
            )
            self._set_if(
                response,
                "command_id",
                getattr(result, "command_id", command_id),
            )

        except Exception as exc:
            rospy.logerr("set_navi failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_navi(self, request):
        response = self._new_response(NaviGetParameter)

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "target_position", [])
            return response

        try:
            result = self._call_service(
                NaviGetParam,
                self.robotname + "/navi_get_param",
            )

            self.target_position = list(result.target_position)
            self._response_code(
                response,
                getattr(result, "returncode_t", "OK"),
            )
            self._set_if(response, "target_position", self.target_position)

        except Exception as exc:
            rospy.logerr("get_navi failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "target_position", [])

        return response

    # ================================================================
    # Parameter bridge: Object Grasping
    # ================================================================
    def set_object_grasping(self, request):
        response = self._new_response(ObjectGraspingSetParameter)
        command_id = "set_object_grasping"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.object_id = request.object_id
            self.grasp_force = request.grasp_force

            lower_req = self._new_request(ObjectGraspingSetParam)
            self._set_if(lower_req, "object_id", self.object_id)
            self._set_if(lower_req, "grasp_force", self.grasp_force)

            result = self._call_service(
                ObjectGraspingSetParam,
                self.robotname + "/object_grasping_set_param",
                lower_req,
            )

            code = getattr(
                result,
                "set_return",
                getattr(result, "returncode_t", "ERROR"),
            )
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)

        except Exception as exc:
            rospy.logerr("set_object_grasping failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_object_grasping(self, request):
        response = self._new_response(ObjectGraspingGetParameter)

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "object_id", "")
            self._set_if(response, "grasp_force", 0.0)
            return response

        try:
            result = self._call_service(
                ObjectGraspingGetParam,
                self.robotname + "/object_grasping_get_param",
            )

            self.object_id = result.object_id
            self.grasp_force = result.grasp_force

            self._response_code(response, "OK")
            self._set_if(response, "object_id", self.object_id)
            self._set_if(response, "grasp_force", self.grasp_force)

        except Exception as exc:
            rospy.logerr("get_object_grasping failed: %s", exc)
            self._response_code(response, "ERROR")
            self._set_if(response, "object_id", "")
            self._set_if(response, "grasp_force", 0.0)

        return response

    # ================================================================
    # Component status
    # ================================================================
    def component_status(self, component_name):
        state_name = self.robotname + "/get_state/" + component_name

        req = self._new_request(ComponentStatus)
        self._set_if(req, "component_name", component_name)

        return self._call_service(
            ComponentStatus,
            state_name,
            req,
            timeout_sec=3.0,
        )

    # ================================================================
    # Event helpers
    # ================================================================
    @staticmethod
    def add_item(target_list, item):
        if item not in target_list:
            target_list.append(item)
            return True
        return False

    @staticmethod
    def remove_item(target_list, item):
        if item in target_list:
            target_list.remove(item)
            return True
        return False

    def notify_event(self, event_id, event_type, subscribe_id):
        msg = Notifyevent()
        msg.event_id = event_id
        msg.event_type = event_type
        msg.subscribe_id = subscribe_id

        self.notify_event_pub.publish(msg)

        rospy.loginfo(
            "event_id=%s, event_type=%s, subscribe_id=%s",
            event_id,
            event_type,
            subscribe_id,
        )

    def event_cb1(self, result):
        if result.event_type != "speech_recognized":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [
            result.event_type,
            result.timestamp,
            result.recognized_text,
        ]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "speech_recognition" + str(self.event_count)

        if self.add_item(self.speech_recognition_list, [event_id, payload]):
            self.notify_event(
                event_id,
                result.event_type,
                self.subscribe_id,
            )

    def event_cb2(self, result):
        if result.event_type != "person_detected":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [
            result.event_type,
            result.timestamp,
            result.number,
        ]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "person_detected" + str(self.event_count)

        if self.add_item(self.person_detected_list, [event_id, payload]):
            self.notify_event(
                event_id,
                result.event_type,
                self.subscribe_id,
            )

    def event_cb3(self, result):
        if result.event_type != "person_localized":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [
            result.event_type,
            result.timestamp,
            result.person_ref,
            result.position_data,
        ]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "person_localized" + str(self.event_count)

        if self.add_item(self.person_position_list, [event_id, payload]):
            self.notify_event(
                event_id,
                result.event_type,
                self.subscribe_id,
            )

    def event_cb4(self, result):
        if result.event_type != "person_identified":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [
            result.event_type,
            result.timestamp,
            result.person_ref,
        ]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "person_identified_" + str(self.event_count)

        if self.add_item(self.person_identi_list, [event_id, payload]):
            self.notify_event(
                event_id,
                result.event_type,
                self.subscribe_id,
            )

    # ================================================================
    # Event detail services
    # ================================================================
    def event_recognized_cb(self, request):
        response = self._new_response(GetEventDetailSpeechRecognized)

        for data in self.speech_recognition_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                self._set_if(response, "recognized_text", data[1][2])
                return response

        self._set_if(response, "timestamp", "")
        self._set_if(response, "recognized_text", "")
        return response

    def event_detected_cb(self, request):
        response = self._new_response(GetEventDetailPersonDetected)

        for data in self.person_detected_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                self._set_if(response, "number", data[1][2])
                return response

        self._set_if(response, "timestamp", "")
        self._set_if(response, "number", 0)
        return response

    def event_local_cb(self, request):
        response = self._new_response(GetEventDetailPersonLocalized)

        for data in self.person_position_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                self._set_if(response, "person_ref", data[1][2])
                self._set_if(response, "position_data", data[1][3])
                return response

        self._set_if(response, "timestamp", "")
        self._set_if(response, "person_ref", "")
        self._set_if(response, "position_data", [])
        return response

    def event_identified_cb(self, request):
        response = self._new_response(GetEventDetailPersonIdentified)

        for data in self.person_identi_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])

                person_ref = data[1][2]
                current = getattr(response, "person_ref", None)

                if isinstance(current, list):
                    response.person_ref = [person_ref]
                else:
                    self._set_if(response, "person_ref", person_ref)

                return response

        self._set_if(response, "timestamp", "")

        if hasattr(response, "person_ref"):
            try:
                response.person_ref = []
            except Exception:
                response.person_ref = ""

        return response

    # ================================================================
    # Event subscribe / unsubscribe
    # ================================================================
    def subscribe_cb(self, request):
        response = self._new_response(Subscribe)

        self.subcount += 1
        self.component_ref = self.BINDCOMP

        event_type = request.event_type
        self.subscribe_id = event_type + str(self.subcount)

        self._destroy_event_subscriptions()

        self.event_subscriber1 = rospy.Subscriber(
            self.robotname + "/event_speechrec",
            NotifySpeechrec,
            self.event_cb1,
            queue_size=10,
        )
        self.event_subscriber2 = rospy.Subscriber(
            self.robotname + "/event_persondetec",
            NotifyPersondetect,
            self.event_cb2,
            queue_size=10,
        )
        self.event_subscriber3 = rospy.Subscriber(
            self.robotname + "/event_localization",
            NotifyLocalrec,
            self.event_cb3,
            queue_size=10,
        )
        self.event_subscriber4 = rospy.Subscriber(
            self.robotname + "/event_identified",
            NotifyIdentified,
            self.event_cb4,
            queue_size=10,
        )

        self._response_code(response, "OK")
        self._set_if(response, "subscribe_id", self.subscribe_id)
        return response

    def _destroy_event_subscriptions(self):
        for attr in (
            "event_subscriber1",
            "event_subscriber2",
            "event_subscriber3",
            "event_subscriber4",
        ):
            sub = getattr(self, attr, None)
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:
                    pass
                setattr(self, attr, None)

    def unsubscribe_cb(self, request):
        response = self._new_response(Unsubscribe)

        try:
            if (
                self.subscribe_id
                and request.subscribe_id != self.subscribe_id
            ):
                self._response_code(response, "ERROR")
                return response

            self._destroy_event_subscriptions()
            self.subscribe_id = ""
            self._response_code(response, "OK")

        except Exception as exc:
            rospy.logerr("unsubscribe failed: %s", exc)
            self._response_code(response, "ERROR")

        return response

    # ================================================================
    # Execute service -> Component action
    # ================================================================
    def execute_cb(self, request):
        response = self._new_response(Execute)

        command = request.command_unit_list
        self.command = command

        if not self.ENGINE_RECEIVABLE or self.BINDCOMP is None:
            self._response_code(response, "ERROR")
            return response

        if command not in {"start", "stop", "suspend", "resume"}:
            self._response_code(response, "ERROR")
            return response

        if self.action_client is None:
            self._response_code(response, "ERROR")
            return response

        try:
            # Preserve the ROS2 unified MainEngine behavior:
            # READY check is required only for start.
            if command == "start":
                c_status = self.component_status(self.BINDCOMP)
                status = getattr(c_status, "status", "ERROR")

                rospy.loginfo("%s is %s", self.BINDCOMP, status)

                if status != "READY":
                    rospy.logwarn(
                        "Cannot start %s: status=%s",
                        self.BINDCOMP,
                        status,
                    )
                    self._response_code(response, "ERROR")
                    return response

            goal = ExecuteGoal()
            goal.command_name = command

            self.action_client.send_goal(
                goal,
                done_cb=self.action_done_callback,
                active_cb=self.action_active_callback,
            )

            # Old MainEngine behavior:
            # return OK once the action request was sent.
            self._response_code(response, "OK")

        except Exception as exc:
            rospy.logerr("execute failed: %s", exc)
            self._response_code(response, "ERROR")

        return response

    def action_active_callback(self):
        rospy.loginfo("Action goal accepted")

    def action_done_callback(self, state, result):
        try:
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(
                    "Action succeeded: success=%s",
                    getattr(result, "success", ""),
                )

            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo("Action canceled/preempted")

            else:
                rospy.logwarn(
                    "Action failed/aborted: state=%s, success=%s",
                    state,
                    getattr(result, "success", ""),
                )

        except Exception as exc:
            rospy.logerr("action_done_callback failed: %s", exc)

    # ================================================================
    # Completed / get_command_result
    # ================================================================
    def completed(self, msg):
        self.command_count += 1

        command_id = msg.command_id + str(self.command_count)
        status = msg.status

        rospy.loginfo("command_id %s is %s", command_id, status)

        item = [command_id, status]

        if self.add_item(self.command_list, item):
            pub = Completed()
            pub.command_id = command_id
            pub.status = status
            self.completed_pub.publish(pub)

    def get_command_result_cb(self, request):
        response = self._new_response(GetCommandResult)

        for data in self.command_list:
            if data[0] == request.command_id:
                self._set_if(response, "results", data[1])
                self._set_if(response, "result", data[1])
                return response

        self._set_if(response, "results", "")
        self._set_if(response, "result", "")
        return response

    def shutdown(self):
        self._destroy_dynamic_parameter_services()
        self._destroy_event_subscriptions()

        if self.action_client is not None:
            try:
                self.action_client.cancel_all_goals()
            except Exception:
                pass
            self.action_client = None


def get_robotfile():
    package_path = rospkg.RosPack().get_path("common_hri_ros1")
    path = os.path.join(package_path, "robot.yaml")

    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main():
    config = get_robotfile()
    robot = config["Robot"]

    robotname = "/" + robot
    node_name = robot + "_Engine"

    rospy.init_node(node_name)

    engine = EngineService(robotname)
    rospy.on_shutdown(engine.shutdown)

    rospy.loginfo(
        "Unified ROS1 MainEngine started. robot=%s",
        robotname,
    )

    rospy.spin()


if __name__ == "__main__":
    main()
