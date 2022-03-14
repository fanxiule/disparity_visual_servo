cmake_minimum_required(VERSION 3.0.2)
project(depth_visual_servo_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

catkin_install_python(PROGRAMS
  script/set_common_param.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()