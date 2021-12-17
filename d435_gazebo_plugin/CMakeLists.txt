cmake_minimum_required(VERSION 3.0.2)
project(d435_gazebo_plugin)
cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp gazebo_ros image_transport camera_info_manager sensor_msgs
    DEPENDS GAZEBO
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(
    ${PROJECT_NAME}
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir include)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
