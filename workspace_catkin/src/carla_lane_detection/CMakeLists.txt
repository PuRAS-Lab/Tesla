cmake_minimum_required(VERSION 3.0.2)
project(carla_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

### Publisher/Subscriber python!
catkin_install_python(PROGRAMS scripts/carla_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/ros_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

