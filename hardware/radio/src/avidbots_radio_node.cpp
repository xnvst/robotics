/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	avidbots_radio_node.cpp
 * @brief	Source File containing the Avidbots Radio Node
 * @author	Feng Cao
 */

// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_radio/radio_manager.h"

int main(int argc, char** argv )
{
  // Creating ROS variables
  ros::init(argc, argv, "avidbots_radio");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  ros::Rate r(10);

  int ret;
  ret = RadioManagerInstance.Init(private_nh, &node);
  if (ret != 0)
  {
    RadioManagerInstance.DeInit();
    return ret;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  RadioManagerInstance.DeInit();

  return 0;
}
