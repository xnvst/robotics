cmake_minimum_required(VERSION 2.8.3)
project(pcl_calibration)

find_package(PCL 1.7.1 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  laser_geometry
  tf_conversions
  roslint
  avidbots_library
)

find_package(TinyXML REQUIRED)

roslint_python()
roslint_cpp()

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp avidbots_library
  DEPENDS TinyXML
#  LIBRARIES pcl_calibration
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)


add_executable(calibrate src/calibrate.cpp)
add_executable(calibrate_laser src/calibrate_laser.cpp)
add_executable(calibrate_laser_wall src/calibrate_laser_wall.cpp)

target_link_libraries(calibrate ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(calibrate_laser avidbots_get_param ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(calibrate_laser_wall ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
    calibrate
    calibrate_laser
    calibrate_laser_wall
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    src/pcl_calibration/calibrate_camera_manual.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)