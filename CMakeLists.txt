cmake_minimum_required(VERSION 2.8.3)
project(ardumotor)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rosserial_arduino
  rosserial_client
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  RPM.msg
)

generate_messages(
  DEPENDENCIES
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motoros
#  CATKIN_DEPENDS message_generation roscpp rosserial_arduino rosserial_client
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware ardumotor ALL)
rosserial_add_client_target(firmware ardumotor-upload)

add_executable(ardumotortest
  src/ardumotortest.cpp
)

target_link_libraries(ardumotortest
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

