#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import actionlib
import rospy
from std_msgs.msg import String
import rospy
import threading
import moveit_commander
from rois_env_ros1.msg import *
from rois_env_ros1.srv import *


class TouchService:
    def __init__(self,robotname):
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)

        self.comp_ref = "Touch"

        self.robotname = robotname

        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, ComponentStatus, self.ComponentStatus)   #
        rospy.loginfo("component status server is ready")


        exe_name = self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, ExecuteAction, self.execute, False)
        print(exe_name)
        self.server.start()

        self.pub = rospy.Publisher(self.robotname + '/completed_command', Completed , queue_size=1)
        # self.pub = rospy.Publisher('/completed', String,queue_size=1)

        self.motion = rospy.ServiceProxy("robot_action", RobotAction)



        self.state = "idle"
        #self.goal_sent = False
        self.current_goal = None

        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする


        self.playback_thread = None




    def set_parameter(self, s_req):
        print("set parameter")
        set_return = "OK"

        return speech_set_paramResponse(set_return)



    def get_parameter(self,g_req):
        print("get_parameter")
        print(self.text)
        return speech_get_paramResponse(self.text)


    def execute(self, goal):
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

        command = goal.command_name
        rospy.loginfo("Received command: %s", command)

        self.feedback = ExecuteFeedback()
        self.result = ExecuteResult()

        if command == "start":
            self.start()

        elif command == "stop":
            self.stop()

        elif command == "suspend":
            self.suspend()

        elif command == "resume":
            self.resume()

        else:
            rospy.loginfo("No valid command received.")
            self.result.success = "False"
            self.server.set_aborted(self.result)



    def start(self):
        self.state = "moving"

        self.motion_thread = threading.Thread(target=self.monitor_playback)
        self.motion_thread.start()

        self.result.success = "True"
        self.server.set_succeeded(self.result)




    def monitor_playback(self):
        rospy.loginfo("Monitoring playback.")

        try:
            rospy.wait_for_service("robot_action", timeout=5.0)
            service = self.motion("approach2")
            print(service.success)

            if service.success==True:
                self.state = "OK"
                self.comp_state = "READY"
            else:
                self.state = "ERROR"
                self.comp_state = "ERROR"

       except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Touch failed: %s", exc)
            self.state = "ERROR"
            self.comp_state = "ERROR"

        finally:
            self.completed_command()


    # 再生が終了したらこのメソッドが呼ばれる
    def completed_command(self):
        rospy.loginfo("Playback completed successfully.")
        pub_data = Completed()
        pub_data.command_id = "Touch"
        pub_data.status = self.state
        print(f"{pub_data.command_id}:{pub_data.status}")
        self.pub.publish(pub_data)

        rospy.loginfo(f'Componemt status: {self.comp_state}')

    # 実機側未対応
    def stop(self):
    rospy.logwarn("Touch stop is not supported.")
    self.result.success = "False"
    self.server.set_aborted(self.result)

    def suspend(self):
        rospy.logwarn("Touch suspend is not supported.")
        self.result.success = "False"
        self.server.set_aborted(self.result)

    def resume(self):
        rospy.logwarn("Touch resume is not supported.")
        self.result.success = "False"
        self.server.set_aborted(self.result)

    ## 音声出力のなごり？
    # def stop(self):
    #     if self.state == "moving":
    #         pygame.mixer.music.stop()
    #         self.state = "stopped"
    #         self.feedback.status = "moving stopped."

    #         pub_data = Completed()
    #         pub_data.command_id = "Touch"
    #         pub_data.status = "stoped"
    #         self.pub.publish(pub_data)

    #         self.result.success = "True"
    #         self.server.set_succeeded(self.result)
    #         self.comp_state = "READY"
    #         rospy.loginfo(f'Componemt status: {self.comp_state}')
    #     else:
    #         rospy.logwarn("No active moving.")
    #         self.feedback.status = "No active goal to stop."
    #         self.result.success = "False"
    #         self.server.set_aborted(self.result)


    # def suspend(self):
    #     if self.state == "moving":

    #         self.state = "suspended"
    #         self.feedback.status = "play suspended."
    #         self.result.success = "True"
    #         self.server.set_succeeded(self.result)
    #     else:
    #         rospy.logwarn("Cannot suspend; not moving.")
    #         self.result.success = "False"
    #         self.server.set_aborted(self.result)

    # def resume(self):
    #     if self.state == "suspended":
    #         pygame.mixer.music.unpause()
    #         self.state = "moving"
    #         self.feedback.status = "moving resumed."
    #         self.result.success = "True"
    #         self.server.set_succeeded(self.result)
    #           # 音楽再生を監視するスレッドを開始
    #         if not self.playback_thread or not self.playback_thread.is_alive():
    #             self.playback_thread = threading.Thread(target=self.monitor_playback)
    #             self.playback_thread.start()
    #     else:
    #         rospy.logwarn("No previous goal to resume.")
    #         self.result.success = "False"
    #         self.server.set_aborted(self.result)

    def ComponentStatus(self, req):
        # 現在の状態を返答
        if (req.component_name == "Touch"):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return ComponentStatusResponse(self.comp_state)
        else:
            pass


    def run(self):
        rospy.loginfo("Service node is running...")
        rospy.spin()



def get_robotfile():
    # 現在のスクリプトの絶対パスを取得
    current_file_path = os.path.abspath(__file__)

    package_relative_path = current_file_path.split('/src/')[1]
    catkin_path = current_file_path.split('/src/')[0]
    package_name = package_relative_path.split('/')[0]

    path = catkin_path +'/src/'+ package_name +"/robot.yaml"

    with open(path, 'r') as file:
        data = yaml.safe_load(file)

    return data


if __name__ == "__main__":
    robot =  get_robotfile()["Robot"]
    robotname =  '/'+ robot
    print(f"Robot name: {robotname}")

    node_name = robot + '_Touch'

    rospy.init_node(node_name)
    service = TouchService(robotname)
    service.run()
