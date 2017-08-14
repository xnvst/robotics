cmake_minimum_required(VERSION 2.8.3)
project(avidbots_mcu_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	avidbots_msgs
  avidbots_library
  avidbots_diagnostics
	roslint)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

find_package(avidbots_msgs REQUIRED)

find_package(avidbots_library REQUIRED)

find_package(avidbots_diagnostics REQUIRED)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${avidbots_msgs_INCLUDE_DIRS}
  ${avidbots_library_INCLUDE_DIRS}
)

add_executable(avidbots_mcu_driver
                src/mcu_driver.cpp
                src/shiny_mcu_driver.cpp
                src/wheely_mcu_driver.cpp
                src/avidbots_can_open.cpp
                src/mcu_driver_node.cpp
                src/mcu_driver_factory.cpp)

target_link_libraries(avidbots_mcu_driver
  ${catkin_LIBRARIES}
  avidbots_socket_can
  avidbots_get_param
)


#############
## Testing ##
#############


#############
## Install ##
#############

install(TARGETS
    avidbots_mcu_driver
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


################
## QT Related ##
################

include(../../qt.cmake)
