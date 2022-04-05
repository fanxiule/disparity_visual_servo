cmake_minimum_required(VERSION 3.0.2)
project(crd_fusion_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_filters
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  StereoImgs.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  script/refine_disp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  script/relay_imgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)