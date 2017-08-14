/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 * @name arduino_serial_node.cpp
 * @brief Source file that launches the arduino serial as a class
 * @author Pablo Molina
 */

// CPP
#include <ros/ros.h>

// ROS

#include "avidbots_arduino/arduino_serial.h"

// ------
//  MAIN
// ------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "AvidbotsArduinoSerial", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ArduinoSerial base_object(&node);
  base_object.Init(private_nh);

  int ret_val = base_object.ReadLoop();

  // it should not come here

  return ret_val;
}
