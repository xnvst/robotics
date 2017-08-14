cmake_minimum_required(VERSION 2.8.3)
project(avidbots_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  avidbots_msgs
  roslint
  avidbots_library)

roslint_cpp()


###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        roscpp
        avidbots_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${avidbots_msgs_INCLUDE_DIRS}
)


add_executable(avidbots_arduino_serial
                src/arduino_serial_node.cpp
                src/arduino_serial.cpp)

target_link_libraries(avidbots_arduino_serial
  ${catkin_LIBRARIES}
  avidbots_get_param
)


#############
## Install ##
#############

install(TARGETS
    avidbots_arduino_serial
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


################
## QT Related ##
################

include(../../qt.cmake)
