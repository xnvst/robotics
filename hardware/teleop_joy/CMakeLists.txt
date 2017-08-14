cmake_minimum_required(VERSION 2.8.3)
project(avidbots_teleop_joy)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  joy
  avidbots_msgs
  avidbots_library
  roslint
)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  joy
  avidbots_msgs
  avidbots_library
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${avidbots_msgs_INCLUDE_DIRS}
)

add_executable(avidbots_teleop_joy 
        src/avidbots_teleop_joy.cpp
        src/avidbots_teleop_joy_node.cpp
        src/avidbots_teleop_joy_control.cpp)

target_link_libraries(
        avidbots_teleop_joy 
        ${catkin_LIBRARIES}
        ${avidbots_msgs_LIBRARIES}
        avidbots_get_param
)

add_executable(avidbots_led_joy
        src/avidbots_led_joy_node.cpp
        src/avidbots_led_joy.cpp)

target_link_libraries(
        avidbots_led_joy
        ${catkin_LIBRARIES}
        ${avidbots_msgs_LIBRARIES}
        avidbots_get_param
)


#############
## Install ##
#############

install(TARGETS
    avidbots_teleop_joy
    avidbots_led_joy
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


################
## QT Related ##
################

include(../../qt.cmake)
