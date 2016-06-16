cmake_minimum_required(VERSION 2.8.3)
project(ardumotor)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rosserial_arduino
  rosserial_client
)

# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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

## Declare a C++ library
# add_library(motoros
#   src/${PROJECT_NAME}/motoros.cpp
# )

## Declare a C++ executable
# add_executable(motoros_node src/motoros_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(motoros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

#############
## Testing ##
#############

