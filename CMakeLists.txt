cmake_minimum_required(VERSION 2.8.3)
project(es5_config_2)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# Declare a catkin package
catkin_package()

# Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joint_space_control src/JointSpaceControl.cpp)
target_link_libraries(joint_space_control ${catkin_LIBRARIES})

add_executable(obstacle_avoidance src/ObstacleAvoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES})