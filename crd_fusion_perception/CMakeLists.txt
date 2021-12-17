cmake_minimum_required(VERSION 3.0.2)
project(crd_fusion_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_filters
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  script/refine_disp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)