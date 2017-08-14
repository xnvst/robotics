cmake_minimum_required(VERSION 2.8.3)
project(avidbots_mc_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###################
## Find packages ##
###################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  avidbots_msgs
  avidbots_library
  avidbots_diagnostics
  roslint
)

find_package(avidbots_msgs REQUIRED)

find_package(avidbots_library REQUIRED)

find_package(avidbots_diagnostics REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread;system)

roslint_python()
roslint_cpp()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp avidbots_library
)

#####################################
## Build libraries and executables ##
#####################################

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${avidbots_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(avidbots_mc_test
                src/test_can_open.cpp
                src/roboteq_nxtgen_can.cpp)

target_link_libraries(avidbots_mc_test
  ${catkin_LIBRARIES}
  avidbots_socket_can
  avidbots_get_param
)

add_executable(mc_driver_node
                src/avidbots_mc_driver_factory.cpp
                src/mc_driver_node.cpp
                src/mc_driver.cpp
                src/kiwi_mc_driver.cpp
                src/bicycle_mc_driver.cpp
                src/roboteq_nxtgen_can.cpp
                src/roboteq_ch_position_motor.cpp
                src/roboteq_sq_position_motor.cpp
                src/roboteq_drive_system_motor.cpp
                src/roboteq_steer_system_motor.cpp
                src/roboteq_cleaning_head_motor.cpp
                src/mc_voltage_helper.cpp
                )

target_link_libraries(mc_driver_node
  ${catkin_LIBRARIES}
  avidbots_socket_can
  avidbots_get_param
)

add_dependencies(mc_driver_node avidbots_msgs_generate_messages_cpp)

add_executable(simulated_free_wheel
                src/simulated_free_wheel.cpp
                )

target_link_libraries(simulated_free_wheel
  ${catkin_LIBRARIES}
  avidbots_get_param
)


#############
## Install ##
#############

install(TARGETS
    mc_driver_node
    simulated_free_wheel
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
    scripts/fake_mc_driver.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)


################
## QT Related ##
################

include(../../qt.cmake)
