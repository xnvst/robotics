cmake_minimum_required(VERSION 2.8.3)
project(avidbots_diagnostics)

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
  pcl_conversions
  pcl_ros
  laser_geometry
  tf_conversions
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
)


## Declare a cpp executable
add_executable(avidbots_diagnostics
               src/avidbots_diagnostics_node.cpp
               src/avidbots_diagnostics.cpp
               src/states_manager.cpp
               src/mcu_states_manager.cpp
               src/mc_states_manager.cpp
               src/urg_states_manager.cpp
               src/camera_states_manager.cpp
               src/sensor_safety_states_manager.cpp
               src/global_diagnostics_manager.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(avidbots_diagnostics
  ${avidbots_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
  avidbots_get_param
)

#############
## Testing ##
#############


#############
## Install ##
#############

install(TARGETS
    avidbots_diagnostics
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################
## QT Related ##
################

include(../qt.cmake)
