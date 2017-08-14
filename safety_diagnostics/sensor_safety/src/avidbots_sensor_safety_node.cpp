/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	avidbots_sensor_safety_node.cpp
 * @brief	Source File containing the Avidbots Sensor Safety node class
 * @author	Feng Cao
 */

// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_sensor_safety/avidbots_sensor_safety.h"

int main(int argc, char** argv )
{
  // Creating ROS variables
  ros::init(argc, argv, "avidbots_sensor_safety");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  AvidbotsSensorSafety* sensor_safety_object_;
  sensor_safety_object_ = new AvidbotsSensorSafety();
  sensor_safety_object_->Init(private_nh, &node);

  ros::spin();

  return 0;
}
