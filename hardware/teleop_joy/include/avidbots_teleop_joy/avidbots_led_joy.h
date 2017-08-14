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
 * @brief	Header file that defines the Avidbots Joystick class
 * @author	Pablo Molina
 */

#ifndef AVIDBOTS_TELEOP_JOY_AVIDBOTS_LED_JOY_H
#define AVIDBOTS_TELEOP_JOY_AVIDBOTS_LED_JOY_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// C++
#include <std_msgs/UInt8.h>
#include <deque>

class AvidbotsLEDJoy
{
public:
  AvidbotsLEDJoy();
  void Init();

private:
  // Helper functions
  void ManagePubSub();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  bool CheckForRisingEdge(const std::deque<bool> & buffer);

  ros::NodeHandle ph_, nh_;

  // Joystick definitions
  int     off_button_,
          yellow_button_,
          green_button_,
          blue_button_;

  // ROS vars
  ros::Publisher  led_status_cmd_pub_;
  ros::Subscriber joy_sub_;

  ros::Timer timer_;

  std_msgs::UInt8 output_msg_;

  // Status variables
  bool  update_output_;

  std::deque<bool> off_status_buff_;
  std::deque<bool> yellow_status_buff_;
  std::deque<bool> green_status_buff_;
  std::deque<bool> blue_status_buff_;
};

#endif  // AVIDBOTS_TELEOP_JOY_AVIDBOTS_LED_JOY_H
