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

#ifndef AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_H
#define AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// C++
#include <std_msgs/UInt8.h>
#include <deque>
#include <string>

#include<avidbots_teleop_joy/avidbots_teleop_joy_control.h>

class AvidbotsTeleopJoy
{
public:
  AvidbotsTeleopJoy();
  void Init();

private:
  // Helper functions
  void ManagePubSub();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  int mapJoystickToFlowCmd(const float& joy_cmd);
  bool CheckForRisingEdge(const std::deque<bool> & buffer);
  double mapTriggerToVelocityMultiplier(const float& trigger_val);

  double ApplyAngularDeadBand(const double& joy_value);

  AvidbotsTeleopJoyControl joy_control_;

  ros::NodeHandle ph_, nh_;

  // Joystick definitions
  int     linear_x_axis_,
          linear_y_axis_,
          angular_axis_,
          vel_deadman_,
          brush_button_,
          vacuum_button_,
          flow_ctrl_button_,
          ch_button_,
          estop_reset_button_,
          m_flow_deadman_,
          m_flow_axis_,
          vel_multiplier_axis_,
          multiplier_input_;

  double  l_scale_,
          a_scale_;

  double  x_dead_band_cut_off_,
          y_dead_band_cut_off_,
          angular_dead_band_cut_off_;

  std::string motion_model_;

  bool    y_use_exp_,
          ang_use_exp_;

  double  vel_multiplier_scaling_slope_,
          vel_multiplier_scaling_y_intercept_;

  double  max_steering_angle_;

  // ROS vars
  ros::Publisher  vel_pub_;
  ros::Publisher  brush_cmd_pub_;
  ros::Publisher  side_brush_cmd_pub_;
  ros::Publisher  vacuum_cmd_pub_;
  ros::Publisher  solen_cmd_pub_;
  ros::Publisher  ch_cmd_pub_;
  ros::Publisher  flow_mode_cmd_pub_;
  ros::Publisher  flow_cmd_pub_;
  ros::Publisher  estop_reset_pub_;

  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_twist_;
  std_msgs::UInt8 last_pub_flow_cmd_;

  bool manual_flow_ctrl_flag_;

  ros::Timer timer_;

  // Status variables
  bool  deadman_pressed_;
  bool  m_flow_deadman_pressed_;
  bool  zero_twist_published_;
  bool  zero_m_flow_published_;
  bool  update_outputs_;
  bool  update_estop_;
  std::deque<bool> brush_status_buff_;
  bool  brush_status_;
  std::deque<bool> vacuum_status_buff_;
  bool  vacuum_status_;
  std::deque<bool> flow_mode_status_buff_;
  bool  flow_mode_status_;
  std::deque<bool> ch_cmd_status_buff_;
  bool  ch_status_;
  std::deque<bool> estop_reset_buff_;
  bool  estop_reset_status;
};

#endif  // AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_H

