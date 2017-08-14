/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	avidbots_base
 * @brief	Source file that defines the joystick node
 * @author	Pablo Molina
 */

#include <ros/ros.h>
#include <avidbots_teleop_joy/avidbots_teleop_joy.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AvidbotsTeleopJoy");
  AvidbotsTeleopJoy joy_object;
  joy_object.Init();

  ros::spin();
}
