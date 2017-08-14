/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	avidbots_teleop_joy_control
 * @brief	Header file that defines the Avidbots Joystick control class.
 *        Allows to change linear or exponential steering for rotation.
 * @author Keshav Iyengar
 */

#ifndef AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_CONTROL_H
#define AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_CONTROL_H

// C++
# include <string>

// ROS
#include <ros/ros.h>

class AvidbotsTeleopJoyControl
{
public:
  AvidbotsTeleopJoyControl();
  void    Init(const double &dead_band_cut_off_x,
               const double &dead_band_cut_off_y,
               const double &dead_band_cut_off_ang,
               const std::string &motion_model,
               const bool &use_y_exp,
               const bool &use_ang_exp);

  double  CalculateControl(const double& joy_value, const int& axis_number);

private:
  double  dead_band_cut_off_x_,
          dead_band_cut_off_y_,
          dead_band_cut_off_ang_;

  std::string motion_model_;

  bool    use_y_exp_,
          use_ang_exp_;

  bool    CheckJoyValue(const double &joy_value_, const double &dead_band_cut_off);

  double  ApplyScaledExpControl(const double &joy_value, const double &dead_band_cut_off);

  double  ApplyScaledLinearControl(const double &joy_value, const double &dead_band_cut_off);
};

#endif  // AVIDBOTS_TELEOP_JOY_AVIDBOTS_TELEOP_JOY_CONTROL_H
