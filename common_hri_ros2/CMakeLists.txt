cmake_minimum_required(VERSION 3.8)
project(common_hri_ros2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  script/Approach.py
  script/Leave.py
  script/Move.py
  script/Navigation.py
  script/Person_Detection.py
  script/Person_Identification.py
  script/Person_Localization.py
  script/Pick.py
  script/Place.py
  script/Speech_Recognition.py
  script/Speech_Synthesis.py
  script/Touch.py
  script/Object_Grasping.py
  script/Object_Release.py
  script/Mainengine_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
	hri.xml
  robot.yaml
  setting.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  yaml
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
