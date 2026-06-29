cmake_minimum_required(VERSION 3.0.2)
project(gen3_lite_hand_follow)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/hand_detector_node.py
  scripts/hand_follow_controller_node.py
  scripts/gesture_gripper_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
