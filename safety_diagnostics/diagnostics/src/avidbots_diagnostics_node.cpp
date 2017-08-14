/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_diagnostics_node.cpp
 * @brief	Source File containing the Avidbots Diagnostics node class
 * @author	Feng Cao
 */

// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_diagnostics/avidbots_diagnostics.h"

int main(int argc, char** argv )
{
  // Creating ROS variables
  ros::init(argc, argv, "avidbots_diagnostics");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  ros::Rate r(10);

  AvidbotsDiagnosticsInstance.Init(private_nh, &node);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    AvidbotsDiagnosticsInstance.PublishDiagnosticsGlobalUpdate();
  }

  return 0;
}
