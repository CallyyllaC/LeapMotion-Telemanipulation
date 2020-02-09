cmake_minimum_required(VERSION 2.8.3)
project(vm_leap)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leap_motion
  message_generation
  roscpp
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

add_executable(vmleap src/vmleap.cpp)
target_link_libraries(vmleap ${catkin_LIBRARIES})
add_dependencies(vmleap vmleap_generate_messages_cpp)
