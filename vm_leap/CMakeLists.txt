cmake_minimum_required(VERSION 2.8.3)
project(vm_leap)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leap_motion
  message_generation
  roscpp
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  vm_leap
)

catkin_package(
  CATKIN_DEPENDS
    leap_motion
    std_msgs
    geometry_msgs
    roscpp
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
