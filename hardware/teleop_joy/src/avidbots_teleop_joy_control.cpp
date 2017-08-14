/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_teleop_joy_control
 * @brief	Source file that defines the Avidbots Joystick control class.
 *        Allows to change linear or exponential steering for rotation.
 * @author Keshav Iyengar, Jake Park
 */

// C++
#include <string>

// Local
#include <avidbots_teleop_joy/avidbots_teleop_joy_control.h>
#include <avidbots_msgs/constants.h>

/**
 * @name  AvidbotsTeleopJoyControl
 * @brief Class constructor
 */
AvidbotsTeleopJoyControl::AvidbotsTeleopJoyControl()
{}

/**
 * @name  Init
 * @brief Initializes the parameters values. Called from avidbots_teleop_joy.cpp
 * @param[in] dead_band_cut_off_x: The deadband value for the x-axis.
 * @param[in] dead_band_cut_off_y: The deadband value for the y-axis.
 * @param[in] dead_band_cut_off_ang: The deadband value for the ang-axis.
 * @param[in] use_omni_drive: Bool for if using an omni drive robot.
 * @param[in] use_y_exp: Bool for if to use exponential control for the y-axis.
 *                       If false, linear control is used.
 * @param[in] use_ang_exp: Bool for if to use exponential control for the ang-axis.
 *                       If false, linear control is used.
 */
void AvidbotsTeleopJoyControl::Init(const double &dead_band_cut_off_x,
                                    const double &dead_band_cut_off_y,
                                    const double &dead_band_cut_off_ang,
                                    const std::string   &motion_model,
                                    const bool   &use_y_exp,
                                    const bool   &use_ang_exp)
{
  dead_band_cut_off_x_     =  dead_band_cut_off_x;
  dead_band_cut_off_y_     =  dead_band_cut_off_y;
  dead_band_cut_off_ang_   =  dead_band_cut_off_ang;

  motion_model_            =  motion_model;
  use_y_exp_               =  use_y_exp;
  use_ang_exp_             =  use_ang_exp;
}

/**
 * @name  CaclulateControl
 * @brief Main method that determines which type of Control to implement.
 * @param[in] joy_value: Value of the joystick input. Range from -1 to 1.
 * @param[in] axis_number: Axis number specific to each axis.
 *                         x-axis   axis_number = 1
 *                         y-axis   axis_number = 0
 *                         ang-axis axis_number = 3
*/
double AvidbotsTeleopJoyControl::CalculateControl(const double& joy_value,
                                                  const int& axis_number)
{
  const int axis_number_ = axis_number;
  const double joy_value_ = joy_value;
  double dead_band_cut_off_ = 0;
  double ret_val = 0;

  //  Set correct dead_band_cut_off value depending on axis.
  if (axis_number_ == 1)
    {
      dead_band_cut_off_ = dead_band_cut_off_x_;
    }

  else if (axis_number_ == 0)
    {
      dead_band_cut_off_ = dead_band_cut_off_y_;
    }

  else if (axis_number_ == 3)
    {
      dead_band_cut_off_ = dead_band_cut_off_ang_;
    }
  else
    {
      ROS_FATAL("Axis number does not match.");
    }

  //  Check if the joystick value is less than the deadband cut off.
  if (CheckJoyValue(joy_value_, dead_band_cut_off_))
    {
      ret_val = 0.0;
    }
  //  If it is not less than cut off, the joystick value is not in deadband.
  else
    {
      //  If it is omni-drive, check if exp steering for y is used and if axis is y.
      if (motion_model_ == KIWI && use_y_exp_ && axis_number_ == 0)
        {
          ret_val = ApplyScaledExpControl(joy_value_, dead_band_cut_off_);
        }
      //  If it is diff drive, check if exp steering for ang is used and if axis is ang.
      else if (motion_model_ == DIFFERENTIAL && use_ang_exp_ && axis_number_ == 3)
        {
          ret_val = ApplyScaledExpControl(joy_value_, dead_band_cut_off_);
        }
      // If it is byicle drive, check if exp steering for y is used and if axis is 0
      else if (motion_model_ == BICYCLE && use_y_exp_ && axis_number_ == 0)
        {
          ret_val = ApplyScaledExpControl(joy_value, dead_band_cut_off_);
        }
      //  All other cases becomes apply linear control.
      else
        {
          ret_val = ApplyScaledLinearControl(joy_value_, dead_band_cut_off_);
        }
    }
  return ret_val;
}

/**
 * @name  CheckJoyValue
 * @brief This method returns a true if the joystick value is less that the deadband cut off.
 * @param[in] joy_value: Value of the joystick input. Range from -1 to 1.
 * @param[in] dead_cut_off: Value of the deadband cut off.
*/
bool AvidbotsTeleopJoyControl::CheckJoyValue(const double& joy_value, const double& dead_band_cut_off)
{
  return std::abs(joy_value) < dead_band_cut_off;
}

/**
 * @name  ApplyScaledExpControl
 * @brief This method returns the scaled exp control value based on the joystick value.
 *        See excel sheet available online (in ticket S1S-99).
 * @param[in] joy_value: Value of the joystick input. Range from -1 to 1.
 * @param[in] dead_cut_off: Value of the deadband cut off.
*/
double AvidbotsTeleopJoyControl::ApplyScaledExpControl(const double &joy_value,
                                                       const double &dead_band_cut_off)
{
  double exp_val_;
  const double dead_band_cut_off_ = dead_band_cut_off;
  exp_val_ = (std::abs(joy_value) * joy_value)/1;

  return (exp_val_ -
          (std::abs(exp_val_) / joy_value)
          * dead_band_cut_off_)
          / (1 - dead_band_cut_off_);
}

/**
 * @name  ApplyLinearExpControl
 * @brief This method returns the scaled exp control value based on the joystick value.
 *        See excel sheet aviable online (in ticket S1S-99).
 * @param[in] joy_value: Value of the joystick input. Range from -1 to 1.
 * @param[in] dead_cut_off: Value of the deadband cut off.
*/
double AvidbotsTeleopJoyControl::ApplyScaledLinearControl(const double &joy_value,
                                                          const double &dead_band_cut_off)
{
  double joy_value_ = joy_value;
  const double dead_band_cut_off_ = dead_band_cut_off;
  return (joy_value_ -
             (std::abs(joy_value_) / joy_value_)
              * dead_band_cut_off_)
             / (1 - dead_band_cut_off_);
}
