/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name  safety_monitor_node
 * @brief Source file that launches the safety monitor node.
 * @author  Keshav Iyengar
 */

// CPP
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// LOCAL
#include <avidbots_safety_monitor/sm_manager.h>

// ------
//  MAIN
// ------
int main(int argc, char** argv)
{
  const char* node_name = "avidbots_safety_monitor";
  ros::init(argc, argv, node_name);

  SMManager* manager_object_;
  manager_object_ = new SMManager();
  manager_object_->Initialize();

  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  delete manager_object_;
  ros::shutdown();
}
