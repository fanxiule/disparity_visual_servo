cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_d435)
cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    gazebo_msgs
    tf
    geometry_msgs
    sensor_msgs
    cv_bridge
    message_filters
    std_msgs
    gazebo_ros
)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    CATKIN_DEPENDS roscpp tf gazebo_msgs geometry_msgs sensor_msgs cv_bridge message_filters std_msgs gazebo_ros
    DEPENDS GAZEBO
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(add_world_frame src/add_world_frame.cpp)
target_link_libraries(add_world_frame ${catkin_LIBRARIES})

add_executable(gen_raw_disp src/stereo_matching.cpp)
target_link_libraries(gen_raw_disp ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS
  script/save_img.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
