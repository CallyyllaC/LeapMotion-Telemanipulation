cmake_minimum_required(VERSION 2.8.3)
project(panda_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  leap_motion
  std_msgs
  geometry_msgs
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  Modified_leap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
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


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
