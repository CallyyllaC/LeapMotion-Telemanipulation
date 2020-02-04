cmake_minimum_required(VERSION 2.8.3)
project(panda_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  leap_motion
  std_msgs
  geometry_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    leap_motion
    std_msgs
    geometry_msgs
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pandaSubscriber src/pandaSubscriber.cpp)
target_link_libraries(pandaSubscriber ${catkin_LIBRARIES})
add_dependencies(pandaSubscriber pandaSubscriber_generate_messages_cpp)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
