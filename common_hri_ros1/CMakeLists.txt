cmake_minimum_required(VERSION 3.0.2)
project(common_hri_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  rois_env_ros1
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    actionlib
    actionlib_msgs
    rois_env_ros1
)

catkin_install_python(
  PROGRAMS
    script/Mainengine_ros1.py
    script/Approach.py
    script/Leave.py
    script/Move.py
    script/Navigation.py
    script/Person_Detection.py
    script/Person_Identification.py
    script/Person_Localization.py
    script/Speech_Recognition.py
    script/Speech_Synthesis.py
    script/Touch.py
    script/Object_Grasping.py
    script/Object_Release.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  hri.xml
  robot.yaml
  setting.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  yaml
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
