cmake_minimum_required(VERSION 2.8.3)
project(avidbots_radio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7.1 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  avidbots_msgs
  avidbots_library
  ecl_time
  roscpp
  roslint
  sensor_msgs
  std_msgs
  cmake_modules
)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES avidbots_safety_controller
  CATKIN_DEPENDS avidbots_msgs avidbots_library roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(avidbots_msgs REQUIRED)

find_package(avidbots_library REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread;system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${avidbots_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/include/net-snmp
)


## Declare a cpp executable
add_executable(avidbots_radio
               src/avidbots_radio_node.cpp
               src/radio_manager.cpp
               src/snmp_manager.cpp
               src/telnet_manager.cpp
               src/util_manager.cpp
              )

set(EXTRALIB_BIN /usr/lib/x86_64-linux-gnu/libsnmp.so)

## Specify libraries to link a library or executable target against
target_link_libraries(avidbots_radio
  ${avidbots_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
  avidbots_get_param
  ${EXTRALIB_BIN}
)

#############
## Testing ##
#############


#############
## Install ##
#############

install(TARGETS
    avidbots_radio
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################
## QT Related ##
################

include(../../qt.cmake)
