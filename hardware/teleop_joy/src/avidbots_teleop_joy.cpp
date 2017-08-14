/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_base
 * @brief	Source file that defines the joystick class; it is modified from turtlebot joystick class
 * @author	Pablo Molina, Jake Park
 */

// ROS
#include <std_msgs/Bool.h>

// C++
#include <deque>

// Local
#include <avidbots_msgs/mcu_output_cmd.h>
#include <avidbots_msgs/constants.h>
#include <avidbots_msgs/topics.h>
#include <avidbots_library/cleaning/flow_mode.h>
#include <avidbots_library/get_param/get_param_util.h>
#include <avidbots_teleop_joy/avidbots_teleop_joy.h>

/**
 * @name  AvidbotsTeleopJoy
 * @brief Class constructor
 */
AvidbotsTeleopJoy::AvidbotsTeleopJoy():
  ph_("~"),
  linear_x_axis_(1),
  linear_y_axis_(0),
  angular_axis_(3),
  vel_deadman_(5),
  brush_button_(3),
  vacuum_button_(1),
  ch_button_(0),
  flow_ctrl_button_(2),
  estop_reset_button_(7),
  l_scale_(0.3),
  a_scale_(0.5),
  brush_status_(false),
  vacuum_status_(false),
  flow_mode_status_(false),
  ch_status_(false),
  estop_reset_status(false),
  m_flow_deadman_(4),
  m_flow_axis_(2),
  vel_multiplier_axis_(5),
  multiplier_input_(3),
  x_dead_band_cut_off_(0.1),
  y_dead_band_cut_off_(0.5),
  angular_dead_band_cut_off_(0.2),
  motion_model_(KIWI),
  y_use_exp_(false),
  ang_use_exp_(false),
  manual_flow_ctrl_flag_(false)

{
  deadman_pressed_        = false;
  zero_twist_published_   = false;
  last_pub_flow_cmd_.data = 0;


  // Initilizing the button status buffer
  for (int i = 0; i < 3 ; i++)
  {
    brush_status_buff_.push_back(false);
    vacuum_status_buff_.push_back(false);
    flow_mode_status_buff_.push_back(false);
    ch_cmd_status_buff_.push_back(false);
    estop_reset_buff_.push_back(false);
  }
}

/**
 * @name  Init
 * @brief Initializes var values
 */
void AvidbotsTeleopJoy::Init()
{
  // Retriving joystick teleop settings
  GetParamUtil::GetParam("/joystick_properties/axis_linear_x", linear_x_axis_, linear_x_axis_);
  GetParamUtil::GetParam("/joystick_properties/axis_linear_y", linear_y_axis_, linear_y_axis_);
  GetParamUtil::GetParam("/joystick_properties/axis_angular", angular_axis_, angular_axis_);
  GetParamUtil::GetParam("/joystick_properties/vel_deadman", vel_deadman_, vel_deadman_);
  GetParamUtil::GetParam("/joystick_properties/brush_button", brush_button_, brush_button_);
  GetParamUtil::GetParam("/joystick_properties/vacuum_button", vacuum_button_, vacuum_button_);
  GetParamUtil::GetParam("/joystick_properties/ch_button", ch_button_, ch_button_);
  GetParamUtil::GetParam("/joystick_properties/flow_ctrl_button", flow_ctrl_button_, flow_ctrl_button_);
  GetParamUtil::GetParam("/joystick_properties/estop_reset_button", estop_reset_button_, estop_reset_button_);
  GetParamUtil::GetParam("/joystick_properties/m_flow_deadman", m_flow_deadman_, m_flow_deadman_);
  GetParamUtil::GetParam("/joystick_properties/m_flow_axis", m_flow_axis_, m_flow_axis_);
  GetParamUtil::GetParam("/joystick_properties/vel_multiplier_axis", vel_multiplier_axis_, vel_multiplier_axis_);
  GetParamUtil::GetParam("/joystick_properties/vel_multiplier", multiplier_input_, multiplier_input_);
  GetParamUtil::GetParam("/joystick_properties/scale_angular", a_scale_, a_scale_);
  GetParamUtil::GetParam("/joystick_properties/scale_linear", l_scale_, l_scale_);
  GetParamUtil::GetParam("/joystick_properties/x_dead_band_cut_off",
                          x_dead_band_cut_off_, x_dead_band_cut_off_);
  GetParamUtil::GetParam("/joystick_properties/y_dead_band_cut_off",
                          y_dead_band_cut_off_, y_dead_band_cut_off_);
  GetParamUtil::GetParam("/joystick_properties/angular_dead_band_cut_off",
                          angular_dead_band_cut_off_, angular_dead_band_cut_off_);
  GetParamUtil::GetParam("/joystick_properties/y_use_exp", y_use_exp_, y_use_exp_);
  GetParamUtil::GetParam("/joystick_properties/ang_use_exp", ang_use_exp_, ang_use_exp_);
  GetParamUtil::GetParam("/joystick_properties/manual_flow_ctrl_flag", manual_flow_ctrl_flag_, manual_flow_ctrl_flag_);
  GetParamUtil::GetParam("/robot_properties/motion_model", motion_model_, motion_model_);
  GetParamUtil::GetParam("/robot_properties/max_steering_angle", max_steering_angle_, 90);

  ManagePubSub();

  joy_control_.Init(x_dead_band_cut_off_, y_dead_band_cut_off_,
                    angular_dead_band_cut_off_, motion_model_,
                    y_use_exp_, ang_use_exp_);

  /* Velocity Multiplier calculations */
  vel_multiplier_scaling_slope_       = (1.0 - multiplier_input_)/2;
  vel_multiplier_scaling_y_intercept_ = (1.0 + multiplier_input_)/2;

  timer_          = nh_.createTimer
        (ros::Duration(0.1), boost::bind(&AvidbotsTeleopJoy::publish, this));
}


/**
 * @name  ManagePubSub
 * @brief Manages the publisher and subscribers
 */
void AvidbotsTeleopJoy::ManagePubSub()
{
  vel_pub_  = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  joy_sub_  = nh_.subscribe<sensor_msgs::Joy>
              ("joy", 10, &AvidbotsTeleopJoy::joyCallback, this);

  brush_cmd_pub_      = nh_.advertise<std_msgs::Bool>
                    (avidbots_topics::cleaning_base_brush, 1, true);

  side_brush_cmd_pub_  = nh_.advertise<std_msgs::Bool>
                    (avidbots_topics::cleaning_base_side_brush, 1, true);

  vacuum_cmd_pub_     = nh_.advertise<std_msgs::Bool>
                    (avidbots_topics::cleaning_base_vacuum, 1, true);

  flow_mode_cmd_pub_  = nh_.advertise<std_msgs::UInt8>
                    (avidbots_topics::cleaning_base_flow_mode, 1, true);

  flow_cmd_pub_       = nh_.advertise<std_msgs::UInt8>
                    (avidbots_topics::cleaning_base_flow_cmd, 1, true);

  ch_cmd_pub_         = nh_.advertise<std_msgs::UInt8>
                    (avidbots_topics::cleaning_base_ch_cmd, 1, true);

  estop_reset_pub_    = nh_.advertise<avidbots_msgs::mcu_output_cmd>
                    (avidbots_topics::mcu_output_cmd_msg, 1, false);
}

/**
 * @name  mapJoystickToFlowCmd
 * @brief Maps the analog input from -1 to 1 into 0-255
 * @param[in] joy_cmd: The joystick command between -1 and 1
 */
int AvidbotsTeleopJoy::mapJoystickToFlowCmd(const float& joy_cmd)
{
  return static_cast<int>(floor( ( (joy_cmd * -127.0) + 128.0 ) ));
}

/**
 * @name  mapTriggerToVelocityMultiplier
 * @brief Maps the analog input from -1 to 1 into 1-multiplier_input_
 * @param[in] joy_cmd: The joystick command between -1 and 1
 */
double AvidbotsTeleopJoy::mapTriggerToVelocityMultiplier(const float& trigger_val)
{
  if (!std::isfinite(trigger_val))
  {
    return 1;
  }

  if (trigger_val > 1 || trigger_val < -1)
  {
    return 1;
  }

  return vel_multiplier_scaling_slope_ * trigger_val + vel_multiplier_scaling_y_intercept_;
}
/**
 * @name  joyCallback
 * @brief The joystick callback
 * @param[in] joy: The joystick msg
 */
void AvidbotsTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Compute velocity
  geometry_msgs::Twist vel;

  vel.angular.z = a_scale_*joy_control_.CalculateControl(joy->axes[angular_axis_], angular_axis_);

  if (motion_model_ == BICYCLE)
  {
    vel.linear.x = mapTriggerToVelocityMultiplier(joy->axes[vel_multiplier_axis_])
        *l_scale_*joy_control_.CalculateControl(joy->axes[linear_x_axis_], linear_x_axis_);
    vel.linear.y = 0;
    vel.angular.z *= max_steering_angle_;
  }
  else
  {
    vel.linear.x  = mapTriggerToVelocityMultiplier(joy->axes[vel_multiplier_axis_])
                    *l_scale_*joy_control_.CalculateControl(joy->axes[linear_x_axis_], linear_x_axis_);

    vel.linear.y  = mapTriggerToVelocityMultiplier(joy->axes[vel_multiplier_axis_])
                    *l_scale_*joy_control_.CalculateControl(joy->axes[linear_y_axis_], linear_y_axis_);
  }

  last_published_twist_ = vel;
  deadman_pressed_ = joy->buttons[vel_deadman_];

  // Adding current status to the buffer
  brush_status_buff_.push_front(static_cast<bool>(joy->buttons[brush_button_]));
  vacuum_status_buff_.push_front(static_cast<bool>(joy->buttons[vacuum_button_]));
  flow_mode_status_buff_.push_front(static_cast<bool>(joy->buttons[flow_ctrl_button_]));
  ch_cmd_status_buff_.push_front(static_cast<bool>(joy->buttons[ch_button_]));
  estop_reset_buff_.push_front(static_cast<bool>(joy->buttons[estop_reset_button_]));

  // Removing last element from the buffer
  brush_status_buff_.pop_back();
  vacuum_status_buff_.pop_back();
  flow_mode_status_buff_.pop_back();
  ch_cmd_status_buff_.pop_back();
  estop_reset_buff_.pop_back();

  if (CheckForRisingEdge(brush_status_buff_))
  {
    brush_status_ = !brush_status_;
    update_outputs_ = true;
  }
  if (CheckForRisingEdge(vacuum_status_buff_))
  {
    vacuum_status_ = !vacuum_status_;
    update_outputs_ = true;
  }
  if (CheckForRisingEdge(flow_mode_status_buff_))
  {
    flow_mode_status_ = !flow_mode_status_;
    update_outputs_ = true;
  }
  if (CheckForRisingEdge(ch_cmd_status_buff_))
  {
    ch_status_ = !ch_status_;
    update_outputs_ = true;
  }
  if (CheckForRisingEdge(estop_reset_buff_))
  {
    estop_reset_status = !estop_reset_status;
    update_outputs_ = true;
    update_estop_   = true;
  }

  // Compute manual flow control command
  if (manual_flow_ctrl_flag_)
  {
    std_msgs::UInt8 flow_cmd;
    flow_cmd.data       = mapJoystickToFlowCmd(joy->axes[m_flow_axis_]);
    last_pub_flow_cmd_  = flow_cmd;
    m_flow_deadman_pressed_ = joy->buttons[m_flow_deadman_];
  }
}
/**
 * @name  publish
 * @brief The publish timer callback - publishes the data if needed
 */
void AvidbotsTeleopJoy::publish()
{
  // Publish outputs if needed
  if (update_outputs_ == true)
  {
    update_outputs_   = false;
    std_msgs::Bool brush_msg;
    brush_msg.data    = brush_status_;
    brush_cmd_pub_.publish(brush_msg);
    side_brush_cmd_pub_.publish(brush_msg);
    std_msgs::Bool vacuum_msg;
    vacuum_msg.data    = vacuum_status_;
    vacuum_cmd_pub_.publish(vacuum_msg);
    if (manual_flow_ctrl_flag_)
    {
      std_msgs::UInt8 flow_ctrl_msg;
      flow_ctrl_msg.data = static_cast<u_int8_t>(flow_mode_status_);
      flow_mode_cmd_pub_.publish(flow_ctrl_msg);
    }
    std_msgs::UInt8 ch_cmd_msg;
    ch_cmd_msg.data =  static_cast<u_int8_t>(ch_status_);
    ch_cmd_pub_.publish(ch_cmd_msg);
    if (update_estop_)
    {
      avidbots_msgs::mcu_output_cmd estop_reset_msg;
      estop_reset_msg.output_num = avidbots_msgs::mcu_output_cmd::kCancelEStop;
      estop_reset_msg.output_value = 0;
      estop_reset_pub_.publish(estop_reset_msg);
      update_estop_ = false;
    }
  }

  // Publish speed if needed
  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_twist_);
    zero_twist_published_ = false;
  }
  else if (!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_ = true;
  }

  // Publish flow command if needed
  if (manual_flow_ctrl_flag_)
  {
    if (m_flow_deadman_pressed_)
    {
      std_msgs::UInt8 flow_mode;
      flow_mode.data = FLOW_MANUAL;
      flow_cmd_pub_.publish(last_pub_flow_cmd_);
      flow_mode_cmd_pub_.publish(flow_mode);
      zero_m_flow_published_ = false;
    }
    else if (!m_flow_deadman_pressed_ && !zero_m_flow_published_)
    {
      std_msgs::UInt8 flow_mode;
      std_msgs::UInt8 flow_cmd;
      flow_mode.data = static_cast<u_int8_t>(flow_mode_status_);
      flow_cmd_pub_.publish(flow_cmd);
      flow_mode_cmd_pub_.publish(flow_mode);
      zero_m_flow_published_ = true;
      last_pub_flow_cmd_.data = 0.0;
    }
  }
}

/**
 * @name  CheckForRisingEdge
 * @brief Checks for the presence of a raising edge on a buffer
 * @param[in] buffer: The buffer of the current button being analyzed
 */
bool AvidbotsTeleopJoy::CheckForRisingEdge(const std::deque<bool> &buffer)
{
  return ((buffer.at(0) == false) && (buffer.at(1) == true)
          && (buffer.at(2) == false));
}
