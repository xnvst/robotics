cmake_minimum_required(VERSION 2.8.3)
project(avidbots_hourmeter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    std_msgs
    avidbots_library
    avidbots_msgs
)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs avidbots_library avidbots_msgs
)

roslint_cpp()

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${avidbots_msgs_INCLUDE_DIRS})

add_executable(avidbots_hourmeter
			src/hourmeter_node.cpp
			src/hourmeter.cpp)

target_link_libraries(avidbots_hourmeter
    ${catkin_LIBRARIES}
    avidbots_get_param
)

#############
## Install ##
#############

install(TARGETS
    avidbots_hourmeter    
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


include(../../qt.cmake)
