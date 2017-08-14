cmake_minimum_required(VERSION 2.8.3)
project(avidbots_safety_monitor)

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
  sensor_msgs
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
add_executable(safety_monitor_node
               src/sm_manager.cpp
               src/sm_node.cpp
               src/sm_ros_interface.cpp
               src/messengers/diagnostics_messenger.cpp
               src/messengers/low_voltage_messenger.cpp
               src/safety_zone/sm_safetyzone_base.cpp
               src/messengers/safetyzone_messenger.cpp
               src/states/dangerzone_state.cpp
               src/states/diagnostics_state.cpp
               src/states/estop_state.cpp
               src/states/exp_safetyzone_state.cpp
               src/states/idle_state.cpp
               src/states/low_voltage_state.cpp
               src/states/state_base.cpp
               src/safety_zone/sm_dangerzone.cpp
               src/safety_zone/sm_expsafetyzone.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(safety_monitor_node
  ${avidbots_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
  avidbots_geometry
  avidbots_get_param
  avidbots_occupancy
  avidbots_safety_zone
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  #Testing targets
  #add_rostest_gtest(safety_zone_test
  #  test/safety_zone_test.launch
  #  test/safety_zone_test.cpp
  #  src/safety_monitor_safetyzone.cpp
  #  src/safety_monitor_helper.cpp
  #)

  #Link in necessary libraries
  #target_link_libraries(safety_zone_test
  #  ${catkin_LIBRARIES}
  #  ${GTEST_LIBRARIES}
  #  avidbots_get_param
  #  avidbots_occupancy
  #  avidbots_safety_zone
  #)

endif()

#############
## Install ##
#############

install(TARGETS
    safety_monitor_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################
## QT Related ##
################

include(../qt.cmake)
