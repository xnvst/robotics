/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	Hourmeter
 * @brief	Node of the hourmeter
 * @author	Pablo Molina
 */

// ROS
#include <ros/ros.h>

// Local
#include "avidbots_hourmeter/hourmeter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avidbots_hourmeter");
  ros::NodeHandle pr_node("~");
  ros::NodeHandle node;

  Hourmeter hour_meter;
  hour_meter.Init(node, pr_node);

  ros::spin();

  return 0;
}
