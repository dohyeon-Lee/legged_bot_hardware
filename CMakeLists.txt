cmake_minimum_required(VERSION 3.0.2)
project(legged_bot_hardware)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosserial_arduino
  rosserial_client
  std_msgs
  tf
  genmsg
)

catkin_package(
  LIBRARIES legged_bot_hardware
  CATKIN_DEPENDS roscpp rospy std_msgs rosserial_arduino rosserial_client tf genmsg
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/leggedlib)

add_executable(${PROJECT_NAME}_node src/legged_bot_hardware_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC leggedlib
  ${catkin_LIBRARIES}  
)

