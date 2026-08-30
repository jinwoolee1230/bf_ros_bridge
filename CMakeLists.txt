cmake_minimum_required(VERSION 3.10)
project(bf_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  mavros_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    diagnostic_msgs
    mavros_msgs
)

catkin_install_python(PROGRAMS
  scripts/bf_bridge_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
