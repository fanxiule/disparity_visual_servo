cmake_minimum_required(VERSION 3.0.2)
project(depth_visual_servo_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()