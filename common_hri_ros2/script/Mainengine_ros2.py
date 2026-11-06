#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Unified ROS2 MainEngine for AIREC / SEED / HSR.

Based on the three ROS1 MainEngine variants and adapted to the current
ROS2 common_hri interfaces.

Robot differences are expected to be expressed by hri.xml / robot.yaml
rather than large robot-name branches in this file.
"""

import os
import yaml
import time
import xml.etree.ElementTree as ET
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rois_env_ros2.action import Execute as ExecuteAction
from rois_env_ros2.srv import *
from rois_env_ros2.srv import Execute as ExecuteService
from rois_env_ros2.msg import (
    Completed,
    Notifyevent,
    NotifyIdentified,
    NotifyLocalrec,
    NotifyPersondetect,
    NotifySpeechrec,
)



class EngineService(Node):
    def __init__(self, robotname: str, node_name: str):
        super().__init__(node_name)

        self.robotname = robotname
        self.ENGINE_RECEIVABLE = False
        self.BINDCOMP: Optional[str] = None
        self.component_ref: Optional[str] = None

        self.cb_group = ReentrantCallbackGroup()

        # -------------------------------
        # Engine state / cached parameters
        # -------------------------------
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
        self.command = ""
        self.pick_and_place_command = ""
        self.shelf_command = ""
        self.target_position = []

        # ROS entities that are created dynamically by bind/subscribe.
        self.action_client: Optional[ActionClient] = None
        self.parameter_get_service = None
        self.parameter_set_service = None
        self.component_status_client = None
        self.lower_parameter_client = None

        self.event_subscriber1 = None
        self.event_subscriber2 = None
        self.event_subscriber3 = None
        self.event_subscriber4 = None

        # -------------------------------
        # Service Application <-> Engine
        # -------------------------------
        self.con = self.create_service(
            SystemInterface, "/connect", self.connect,
            callback_group=self.cb_group,
        )
        self.discon = self.create_service(
            SystemInterface, "/disconnect", self.disconnect,
            callback_group=self.cb_group,
        )
        self.bind = self.create_service(
            BindAny, "/bind_any", self.bind_any,
            callback_group=self.cb_group,
        )
        self.release_srv = self.create_service(
            Release, "/release", self.release,
            callback_group=self.cb_group,
        )
        self.execute_srv = self.create_service(
            ExecuteService, "/execute", self.execute_cb,
            callback_group=self.cb_group,
        )
        self.subscribe_srv = self.create_service(
            Subscribe, "/subscribe", self.subscribe_cb,
            callback_group=self.cb_group,
        )
        self.unsubscribe_srv = self.create_service(
            Unsubscribe, "/unsubscribe", self.unsubscribe_cb,
            callback_group=self.cb_group,
        )

        self.eventdetailservice1 = self.create_service(
            GetEventDetailSpeechRecognized,
            "/recognized_event_detail",
            self.event_recognized_cb,
            callback_group=self.cb_group,
        )
        self.eventdetailservice2 = self.create_service(
            GetEventDetailPersonDetected,
            "/detected_event_detail",
            self.event_detected_cb,
            callback_group=self.cb_group,
        )
        self.eventdetailservice3 = self.create_service(
            GetEventDetailPersonLocalized,
            "/local_event_detail",
            self.event_local_cb,
            callback_group=self.cb_group,
        )
        self.eventdetailservice4 = self.create_service(
            GetEventDetailPersonIdentified,
            "/identified_event_detail",
            self.event_identified_cb,
            callback_group=self.cb_group,
        )

        self.get_command_result_service = self.create_service(
            GetCommandResult,
            "/get_command_result",
            self.get_command_result_cb,
            callback_group=self.cb_group,
        )

        # -------------------------------
        # Component -> Engine -> App topics
        # -------------------------------
        self.completed_sub = self.create_subscription(
            Completed,
            self.robotname + "/completed_command",
            self.completed,
            10,
            callback_group=self.cb_group,
        )
        self.completed_pub = self.create_publisher(Completed, "/completed", 10)
        self.notify_event_pub = self.create_publisher(Notifyevent, "/notify_event", 10)

        self.get_logger().info(
            f"Engine Service is ready. robot={self.robotname}"
        )

    # ================================================================
    # Utility
    # ================================================================
    @staticmethod
    # srv(odj)のなかに求めてる名前の型(field)があるか確認してからいれる
    def _set_if(obj, field, value):
        if hasattr(obj, field):
            setattr(obj, field, value)

    @staticmethod
    # 返り値ばらばら対策
    def _response_code(response, value):
        # Existing interfaces use a few different historical names.
        for field in ("returncode_t", "return_t", "set_return", "result"):
            if hasattr(response, field):
                setattr(response, field, value)
                return

    # serviceの答えを待つ（同期っぽく）
    def _wait_future(self, future, timeout_sec=5.0):
        """Wait for a Future while MultiThreadedExecutor handles callbacks."""
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                raise TimeoutError("ROS2 future timed out")
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError("ROS2 future was interrupted")
        exc = future.exception()
        if exc is not None:
            raise exc
        return future.result()

    # Service Client生成〜呼び出し〜終了待ち〜破棄
    def _call_service(self, srv_type, service_name, request, timeout_sec=5.0):
        client = self.create_client(
            srv_type,
            service_name,
            callback_group=self.cb_group,
        )
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"Service not available: {service_name}")
            return self._wait_future(client.call_async(request), timeout_sec)
        finally:
            self.destroy_client(client)

    def is_service_available(self, service_name):
        names = [name for name, _types in self.get_service_names_and_types()]
        return service_name in names

    def is_action_server_active(self, action_name):
        client = ActionClient(
            self,
            ExecuteAction,
            action_name,
            callback_group=self.cb_group,
        )
        try:
            return client.wait_for_server(timeout_sec=1.0)
        finally:
            client.destroy()

    def get_component(self):
        """Read HRIComponent entries from installed hri.xml."""
        try:
            package_share = get_package_share_directory("common_hri_ros2")
            file_path = os.path.join(package_share, "hri.xml")
        except Exception:
            file_path = os.path.join(os.path.dirname(__file__), "hri.xml")

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
            # Preserve old behavior but also accept the actual urn-style text.
            if "::" in text:
                name = text.split("::")[-1]
            else:
                name = text.replace("rois:HRIComponent:", "")
            components.append(name)

        return components

    def _destroy_dynamic_parameter_services(self):
        if self.parameter_get_service is not None:
            self.destroy_service(self.parameter_get_service)
            self.parameter_get_service = None
        if self.parameter_set_service is not None:
            self.destroy_service(self.parameter_set_service)
            self.parameter_set_service = None

    def _create_action_client(self, component_ref):
        if self.action_client is not None:
            self.action_client.destroy()

        exe_name = self.robotname + "/execute/" + component_ref
        self.action_client = ActionClient(
            self,
            ExecuteAction,
            exe_name,
            callback_group=self.cb_group,
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.action_client.destroy()
            self.action_client = None
            raise RuntimeError(f"Action server not available: {exe_name}")

        self.get_logger().info(f"Action client ready: {exe_name}")

    # ================================================================
    # System Interface
    # ================================================================
    def connect(self, request, response):
        if request.connect == "connect":
            self.ENGINE_RECEIVABLE = True
            self.get_logger().info("Engine is receivable")
            self._response_code(response, "OK")
        else:
            self._response_code(response, "ERROR")
        return response

    def disconnect(self, request, response):
        if request.connect == "disconnect":
            self.ENGINE_RECEIVABLE = False
            self.get_logger().info("Engine is not receivable")
            self._response_code(response, "OK")
        else:
            self._response_code(response, "ERROR")
        return response

    # ================================================================
    # Command Interface: bind / release
    # ================================================================
    def bind_any(self, request, response):
        try:
            self.component_ref_list = self.get_component()
            self.component_ref = request.component_ref

            self.get_logger().info(
                f"bind_any: requested={self.component_ref}, "
                f"available={self.component_ref_list}"
            )

            if not self.ENGINE_RECEIVABLE:
                self._response_code(response, "ERROR")
                return response

            if self.BINDCOMP is not None:
                self.get_logger().warning(f"Already bound: {self.BINDCOMP}")
                self._response_code(response, "ERROR")
                return response

            if self.component_ref not in self.component_ref_list:
                self.get_logger().warning(
                    f"HRI-Component does not exist: {self.component_ref}"
                )
                self._response_code(response, "ERROR")
                return response
        
            get_name = "/get_parameter/" + self.component_ref
            set_name = "/set_parameter/" + self.component_ref

            self._destroy_dynamic_parameter_services()

            # --------------------------------------------------------
            # Components with Engine-level get/set parameter services
            # --------------------------------------------------------
            if self.component_ref == "Speech_Synthesis":
                self.text = "hello"
                self.parameter_get_service = self.create_service(
                    SpeechGetParameter, get_name, self.get_speech,
                    callback_group=self.cb_group,
                )
                self.parameter_set_service = self.create_service(
                    SpeechSetParameter, set_name, self.set_speech,
                    callback_group=self.cb_group,
                )

            elif self.component_ref == "Speech_Recognition":
                self.language = "japanese"
                self.parameter_get_service = self.create_service(
                    SpeechRecognitionGetParameter,
                    get_name,
                    self.get_s_recog,
                    callback_group=self.cb_group,
                )
                self.parameter_set_service = self.create_service(
                    SpeechRecognitionSetParameter,
                    set_name,
                    self.set_s_recog,
                    callback_group=self.cb_group,
                )

            elif self.component_ref == "Move":
                self.line = [500, 0, 0]
                self.curve = []
                self.parameter_get_service = self.create_service(
                    MoveGetParameter, get_name, self.get_move,
                    callback_group=self.cb_group,
                )
                self.parameter_set_service = self.create_service(
                    MoveSetParameter, set_name, self.set_move,
                    callback_group=self.cb_group,
                )

            elif self.component_ref == "Approach":
                self.command = "approach1"
                self.parameter_get_service = self.create_service(
                    ApproachGetParameter, get_name, self.get_approach,
                    callback_group=self.cb_group,
                )
                self.parameter_set_service = self.create_service(
                    ApproachSetParameter, set_name, self.set_approach,
                    callback_group=self.cb_group,
                )

            # elif self.component_ref == "Pick_and_Place":
            #     self.pick_and_place_command = ""
            #     self.parameter_get_service = self.create_service(
            #         ApproachGetParameter,
            #         get_name,
            #         self.get_pick_and_place,
            #         callback_group=self.cb_group,
            #     )
            #     self.parameter_set_service = self.create_service(
            #         ApproachSetParameter,
            #         set_name,
            #         self.set_pick_and_place,
            #         callback_group=self.cb_group,
            #     )

            # elif self.component_ref == "Shelf":
            #     self.shelf_command = ""
            #     self.parameter_get_service = self.create_service(
            #         ApproachGetParameter, get_name, self.get_shelf,
            #         callback_group=self.cb_group,
            #     )
            #     self.parameter_set_service = self.create_service(
            #         ApproachSetParameter, set_name, self.set_shelf,
            #         callback_group=self.cb_group,
            #     )

            elif self.component_ref == "Navigation":
                self.target_position = []
                self.parameter_get_service = self.create_service(
                    NaviGetParameter, get_name, self.get_navi,
                    callback_group=self.cb_group,
                )
                self.parameter_set_service = self.create_service(
                    NaviSetParameter, set_name, self.set_navi,
                    callback_group=self.cb_group,
                )

            elif self.component_ref == "Object_Grasping":
                self.object_id = ""
                self.grasp_force = 0.0
                self.parameter_get_service = self.create_service(
                    ObjectGraspingGetParameter,
                    get_name,
                    self.get_object_grasping,
                    callback_group=self.cb_group,
                )

                self.parameter_set_service = self.create_service(
                    ObjectGraspingSetParameter,
                    set_name,
                    self.set_object_grasping,
                    callback_group=self.cb_group,
                )

            # Action-only components require no parameter service here.
            # Includes old MainEngine components and current common_hri ones.
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
                self.get_logger().warning(
                    f"No bind rule implemented for {self.component_ref}"
                )
                self._response_code(response, "ERROR")
                return response

            self._create_action_client(self.component_ref)
            self.BINDCOMP = self.component_ref
            self.get_logger().info(f"Bind {self.component_ref}")
            self._response_code(response, "OK")

        except Exception as exc:
            self.get_logger().error(f"bind_any failed: {exc}")
            self._destroy_dynamic_parameter_services()
            self.BINDCOMP = None
            self.component_ref = None
            self._response_code(response, "ERROR")

        return response

    def release(self, request, response):
        try:
            self._destroy_dynamic_parameter_services()

            if self.action_client is not None:
                self.action_client.destroy()
                self.action_client = None

            self.BINDCOMP = None
            self.component_ref = None
            self._response_code(response, "OK")
        except Exception as exc:
            self.get_logger().error(f"release failed: {exc}")
            self._response_code(response, "ERROR")
        return response

    # ================================================================
    # Parameter bridge: Speech Synthesis
    # ================================================================
    def set_speech(self, request, response):
        command_id = "set_speech_synthesis"
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.text = request.speech_text
            lower_req = SpeechSetParam.Request()
            lower_req.speech_text = self.text
            result = self._call_service(
                SpeechSetParam,
                self.robotname + "/speech_set_param",
                lower_req,
            )
            code = getattr(result, "set_return", "ERROR")
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)
        except Exception as exc:
            self.get_logger().error(f"set_speech failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
        return response

    def get_speech(self, request, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "speech_text", "")
            return response

        try:
            result = self._call_service(
                SpeechGetParam,
                self.robotname + "/speech_get_param",
                SpeechGetParam.Request(),
            )
            self.text = result.speech_text
            self._response_code(response, "OK")
            self._set_if(response, "speech_text", self.text)
        except Exception as exc:
            self.get_logger().error(f"get_speech failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "speech_text", "")
        return response

    # ================================================================
    # Parameter bridge: Speech Recognition
    # ================================================================
    def set_s_recog(self, request, response):
        command_id = "set_speech_recognition"
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.language = request.languages
            lower_req = SpeechRecognitionSetParam.Request()
            lower_req.languages = self.language
            result = self._call_service(
                SpeechRecognitionSetParam,
                self.robotname + "/s_recognition_set_param",
                lower_req,
            )
            code = getattr(result, "set_return", "ERROR")
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)
        except Exception as exc:
            self.get_logger().error(f"set_s_recog failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
        return response

    def get_s_recog(self, request, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "recognizable_languages", [])
            self._set_if(response, "languages", "")
            return response

        try:
            result = self._call_service(
                SpeechRecognitionGetParam,
                self.robotname + "/s_recognition_get_param",
                SpeechRecognitionGetParam.Request(),
            )
            self.language = result.languages
            self.recog_language = list(result.recognizable_languages)
            self._response_code(response, "OK")
            self._set_if(response, "recognizable_languages", self.recog_language)
            self._set_if(response, "languages", self.language)
        except Exception as exc:
            self.get_logger().error(f"get_s_recog failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "recognizable_languages", [])
            self._set_if(response, "languages", "")
        return response

    # ================================================================
    # Parameter bridge: Move
    # ================================================================
    def set_move(self, request, response):
        command_id = "set_move"
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.line = list(request.line)
            self.curve = list(request.curve)

            # Current ROS2 common_hri/Move exposes MoveSetParameter directly.
            lower_req = MoveSetParam.Request()
            lower_req.line = self.line
            lower_req.curve = self.curve
            if hasattr(request, "time") and hasattr(lower_req, "time"):
                lower_req.time = request.time

            result = self._call_service(
                MoveSetParam,
                self.robotname + "/move_set_param",
                lower_req,
            )
            code = getattr(result, "returncode_t", "ERROR")
            self._response_code(response, code)
            self._set_if(
                response,
                "command_id",
                getattr(result, "command_id", command_id),
            )
        except Exception as exc:
            self.get_logger().error(f"set_move failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
        return response

    def get_move(self, request, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "line", [])
            self._set_if(response, "curve", [])
            return response

        try:
            result = self._call_service(
                MoveGetParam,
                self.robotname + "/move_get_param",
                MoveGetParam.Request(),
            )
            self.line = list(result.line)
            self.curve = list(result.curve)
            self._response_code(response, getattr(result, "returncode_t", "OK"))
            self._set_if(response, "line", self.line)
            self._set_if(response, "curve", self.curve)
            if hasattr(result, "time"):
                self._set_if(response, "time", result.time)
        except Exception as exc:
            self.get_logger().error(f"get_move failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "line", [])
            self._set_if(response, "curve", [])
        return response

    # ================================================================
    # Parameter bridge: Approach / Pick_and_Place / Shelf
    # ================================================================
    def _set_approach_like(self, value, lower_service_name, command_id, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            lower_req = ApproachSetParam.Request()
            lower_req.approach_command = value
            result = self._call_service(
                ApproachSetParam,
                lower_service_name,
                lower_req,
            )
            code = getattr(result, "set_return", "ERROR")
            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)
        except Exception as exc:
            self.get_logger().error(f"{command_id} failed: {exc}")
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
                ApproachGetParam.Request(),
            )
            value = result.approach_command
            self._response_code(response, "OK")
            self._set_if(response, "approach_command", value)
            return response, value
        except Exception as exc:
            self.get_logger().error(f"get parameter failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "approach_command", "")
            return response, ""

    def set_approach(self, request, response):
        self.command = request.approach_command
        return self._set_approach_like(
            self.command,
            self.robotname + "/approach_set_param",
            "set_approach",
            response,
        )

    def get_approach(self, request, response):
        response, self.command = self._get_approach_like(
            self.robotname + "/approach_get_param",
            response,
        )
        return response

    def set_pick_and_place(self, request, response):
        self.pick_and_place_command = request.approach_command
        return self._set_approach_like(
            self.pick_and_place_command,
            self.robotname + "/pick_and_place_set_param",
            "set_pick_and_place",
            response,
        )

    def get_pick_and_place(self, request, response):
        response, self.pick_and_place_command = self._get_approach_like(
            self.robotname + "/pick_and_place_get_param",
            response,
        )
        return response

    def set_shelf(self, request, response):
        self.shelf_command = request.approach_command
        return self._set_approach_like(
            self.shelf_command,
            self.robotname + "/shelf_set_param",
            "set_shelf",
            response,
        )

    def get_shelf(self, request, response):
        response, self.shelf_command = self._get_approach_like(
            self.robotname + "/shelf_get_param",
            response,
        )
        return response

    # ================================================================
    # Parameter bridge: Navigation
    # ================================================================
    def set_navi(self, request, response):
        command_id = "set_navigation"
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.target_position = list(request.target_position)
            lower_req = NaviSetParam.Request()
            lower_req.target_position = self.target_position
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
            self.get_logger().error(f"set_navi failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
        return response

    def get_navi(self, request, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "target_position", [])
            return response

        try:
            result = self._call_service(
                NaviGetParam,
                self.robotname + "/navi_get_param",
                NaviGetParam.Request(),
            )
            self.target_position = list(result.target_position)
            self._response_code(
                response,
                getattr(result, "returncode_t", "OK"),
            )
            self._set_if(response, "target_position", self.target_position)
        except Exception as exc:
            self.get_logger().error(f"get_navi failed: {exc}")
            self._response_code(response, "ERROR")
            self._set_if(response, "target_position", [])
        return response

    # ================================================================
    # Parameter bridge: Object_Grasping
    # ================================================================
    def set_object_grasping(self, request, response):
        command_id = "set_object_grasping"

        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)
            return response

        try:
            self.object_id = request.object_id
            self.grasp_force = request.grasp_force

            lower_req = ObjectGraspingSetParam.Request()
            lower_req.object_id = self.object_id
            lower_req.grasp_force = self.grasp_force

            result = self._call_service(
                ObjectGraspingSetParam,
                self.robotname + "/object_grasping_set_param",
                lower_req,
            )

            code = getattr(result, "set_return", "ERROR")

            self._response_code(response, code)
            self._set_if(response, "command_id", command_id)

        except Exception as exc:
            self.get_logger().error(
                f"set_object_grasping failed: {exc}"
            )
            self._response_code(response, "ERROR")
            self._set_if(response, "command_id", command_id)

        return response

    def get_object_grasping(self, request, response):
        if not self.ENGINE_RECEIVABLE:
            self._response_code(response, "ERROR")
            self._set_if(response, "object_id", "")
            self._set_if(response, "grasp_force", 0.0)
            return response

        try:
            result = self._call_service(
                ObjectGraspingGetParam,
                self.robotname + "/object_grasping_get_param",
                ObjectGraspingGetParam.Request(),
            )

            self.object_id = result.object_id
            self.grasp_force = result.grasp_force

            self._response_code(response, "OK")
            self._set_if(response, "object_id", self.object_id)
            self._set_if(response, "grasp_force", self.grasp_force)

        except Exception as exc:
            self.get_logger().error(
                f"get_object_grasping failed: {exc}"
            )

            self._response_code(response, "ERROR")
            self._set_if(response, "object_id", "")
            self._set_if(response, "grasp_force", 0.0)

        return response

    # ================================================================
    # Component status
    # ================================================================
    def component_status(self, component_name):
        state_name = self.robotname + "/get_state/" + component_name
        req = ComponentStatus.Request()
        if hasattr(req, "component_name"):
            req.component_name = component_name

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
        self.get_logger().info(
            f"event_id={event_id}, event_type={event_type}, "
            f"subscribe_id={subscribe_id}"
        )

    def event_cb1(self, result):
        if result.event_type != "speech_recognized":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [result.event_type, result.timestamp, result.recognized_text]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "speech_recognition" + str(self.event_count)

        if self.add_item(self.speech_recognition_list, [event_id, payload]):
            self.notify_event(event_id, result.event_type, self.subscribe_id)

    def event_cb2(self, result):
        if result.event_type != "person_detected":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [result.event_type, result.timestamp, result.number]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "person_detected" + str(self.event_count)

        if self.add_item(self.person_detected_list, [event_id, payload]):
            self.notify_event(event_id, result.event_type, self.subscribe_id)

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
            self.notify_event(event_id, result.event_type, self.subscribe_id)

    def event_cb4(self, result):
        if result.event_type != "person_identified":
            return
        if result.timestamp == self.last_timestamp:
            return

        payload = [result.event_type, result.timestamp, result.person_ref]
        self.last_timestamp = result.timestamp
        self.event_count += 1
        event_id = "person_identified_" + str(self.event_count)

        if self.add_item(self.person_identi_list, [event_id, payload]):
            self.notify_event(event_id, result.event_type, self.subscribe_id)

    # ================================================================
    # Event detail services
    # ================================================================
    def event_recognized_cb(self, request, response):
        for data in self.speech_recognition_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                self._set_if(response, "recognized_text", data[1][2])
                return response

        self._set_if(response, "timestamp", "")
        self._set_if(response, "recognized_text", "")
        return response

    def event_detected_cb(self, request, response):
        for data in self.person_detected_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                self._set_if(response, "number", data[1][2])
                return response

        self._set_if(response, "timestamp", "")
        self._set_if(response, "number", 0)
        return response

    def event_local_cb(self, request, response):
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

    def event_identified_cb(self, request, response):
        for data in self.person_identi_list:
            if data[0] == request.event_id:
                self._set_if(response, "timestamp", data[1][1])
                person_ref = data[1][2]
                # Old response used [person_ref]. Keep compatibility with arrays.
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
    def subscribe_cb(self, request, response):
        self.subcount += 1
        self.component_ref = self.BINDCOMP

        event_type = request.event_type
        self.subscribe_id = event_type + str(self.subcount)

        # Avoid duplicate subscriptions when subscribe is called repeatedly.
        self._destroy_event_subscriptions()

        self.event_subscriber1 = self.create_subscription(
            NotifySpeechrec,
            self.robotname + "/event_speechrec",
            self.event_cb1,
            10,
            callback_group=self.cb_group,
        )
        self.event_subscriber2 = self.create_subscription(
            NotifyPersondetect,
            self.robotname + "/event_persondetec",
            self.event_cb2,
            10,
            callback_group=self.cb_group,
        )
        self.event_subscriber3 = self.create_subscription(
            NotifyLocalrec,
            self.robotname + "/event_localization",
            self.event_cb3,
            10,
            callback_group=self.cb_group,
        )
        self.event_subscriber4 = self.create_subscription(
            NotifyIdentified,
            self.robotname + "/event_identified",
            self.event_cb4,
            10,
            callback_group=self.cb_group,
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
                self.destroy_subscription(sub)
                setattr(self, attr, None)

    def unsubscribe_cb(self, request, response):
        try:
            if self.subscribe_id and request.subscribe_id != self.subscribe_id:
                self._response_code(response, "ERROR")
                return response

            self._destroy_event_subscriptions()
            self.subscribe_id = ""
            self._response_code(response, "OK")
        except Exception as exc:
            self.get_logger().error(f"unsubscribe failed: {exc}")
            self._response_code(response, "ERROR")
        return response

    # ================================================================
    # Execute -> Component Action
    # ================================================================
    def execute_cb(self, request, response):
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
            # Preserve old behavior: READY check is required only for start.
            if command == "start":
                c_status = self.component_status(self.BINDCOMP)
                status = getattr(c_status, "status", "ERROR")
                self.get_logger().info(f"{self.BINDCOMP} is {status}")

                if status != "READY":
                    self.get_logger().warning(
                        f"Cannot start {self.BINDCOMP}: status={status}"
                    )
                    self._response_code(response, "ERROR")
                    return response

            if not self.action_client.wait_for_server(timeout_sec=3.0):
                raise RuntimeError("Action server is not available")

            goal = ExecuteAction.Goal()
            goal.command_name = command

            send_future = self.action_client.send_goal_async(goal)
            send_future.add_done_callback(self.goal_response_callback)

            # Old MainEngine returns OK once the action request has been sent.
            self._response_code(response, "OK")

        except Exception as exc:
            self.get_logger().error(f"execute failed: {exc}")
            self._response_code(response, "ERROR")

        return response

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning("Action goal rejected")
                return

            self.get_logger().info("Action goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.action_done_callback)
        except Exception as exc:
            self.get_logger().error(f"goal_response_callback failed: {exc}")

    def action_done_callback(self, future):
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Action succeeded: success={getattr(result, 'success', '')}"
                )
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Action canceled")
            else:
                self.get_logger().warning(
                    f"Action failed/aborted: status={status}, "
                    f"success={getattr(result, 'success', '')}"
                )
        except Exception as exc:
            self.get_logger().error(f"action_done_callback failed: {exc}")

    # ================================================================
    # Completed / get_command_result
    # ================================================================
    def completed(self, msg):
        self.command_count += 1
        command_id = msg.command_id + str(self.command_count)
        status = msg.status

        self.get_logger().info(f"command_id {command_id} is {status}")

        item = [command_id, status]
        if self.add_item(self.command_list, item):
            pub = Completed()
            pub.command_id = command_id
            pub.status = status
            self.completed_pub.publish(pub)

    def get_command_result_cb(self, request, response):
        for data in self.command_list:
            if data[0] == request.command_id:
                self._set_if(response, "results", data[1])
                self._set_if(response, "result", data[1])
                return response

        self._set_if(response, "results", "")
        self._set_if(response, "result", "")
        return response

    def destroy_node(self):
        self._destroy_dynamic_parameter_services()
        self._destroy_event_subscriptions()
        if self.action_client is not None:
            self.action_client.destroy()
            self.action_client = None
        super().destroy_node()


def get_robotfile():
    package_share = get_package_share_directory("common_hri_ros2")
    path = os.path.join(package_share, "robot.yaml")

    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)

    config = get_robotfile()
    robot = config["Robot"]
    robotname = "/" + robot
    node_name = robot + "_Engine"

    node = EngineService(robotname, node_name)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
