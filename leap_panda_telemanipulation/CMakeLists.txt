cmake_minimum_required(VERSION 2.8.3)
project(leap_panda_telemanipulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leap_motion
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  Modified_leap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  leap_panda_telemanipulation
)

catkin_package(
  CATKIN_DEPENDS
    leap_motion
    std_msgs
    geometry_msgs
    roscpp
    rospy
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LeaptoPanda src/LeaptoPanda.cpp)
target_link_libraries(LeaptoPanda ${catkin_LIBRARIES})
add_dependencies(LeaptoPanda LeaptoPanda_generate_messages_cpp)

install(PROGRAMS scripts/LeapMoveGroup.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
