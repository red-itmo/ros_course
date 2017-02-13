cmake_minimum_required(VERSION 2.8.3)
project(ros_course)

find_package(catkin REQUIRED
    roscpp
    rospy
    geometry_msgs
    turtlesim
)

catkin_package()

# include_directories(include)

add_executable(controller_node_cpp src/controller_node.cpp)

target_link_libraries(controller_node_cpp
    ${catkin_LIBRARIES}
)
