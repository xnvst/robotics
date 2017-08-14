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
 * @brief	Source file that defines the joystick class
 * @author	Pablo Molina - modified from turtlebot joystick class
 */

#include <deque>
// ROS
#include <avidbots_teleop_joy/avidbots_led_joy.h>
#include <avidbots_library/socket_can/avidbots_can_open_constants.h>
#include <avidbots_msgs/topics.h>
#include <std_msgs/Bool.h>

// Local
#include <avidbots_library/get_param/get_param_util.h>

/**
 * @name  AvidbotsTeleopJoy
 * @brief Class constructor
 */
AvidbotsLEDJoy::AvidbotsLEDJoy():
  ph_("~"),
  off_button_(3),
  yellow_button_(1),
  green_button_(0),
  blue_button_(2)
{
  // Initilizing the button status buffer
  for (int i = 0; i < 3 ; i++)
  {
    off_status_buff_.push_back(false);
    yellow_status_buff_.push_back(false);
    green_status_buff_.push_back(false);
    blue_status_buff_.push_back(false);
  }
}

/**
 * @name  Init
 * @brief Initializes var values
 */
void AvidbotsLEDJoy::Init()
{
  ManagePubSub();
  timer_          = nh_.createTimer
        (ros::Duration(0.1), boost::bind(&AvidbotsLEDJoy::publish, this));
}


/**
 * @name  ManagePubSub
 * @brief Manages the publisher and subscribers
 */
void AvidbotsLEDJoy::ManagePubSub()
{
  joy_sub_  = nh_.subscribe<sensor_msgs::Joy>
              ("joy_led", 10, &AvidbotsLEDJoy::joyCallback, this);

  led_status_cmd_pub_      = nh_.advertise<std_msgs::UInt8>
                    (avidbots_topics::cleaning_base_led_status, 1, true);
}


/**
 * @name  joyCallback
 * @brief The joystick callback
 * @param[in] joy: The joystick msg
 */
void AvidbotsLEDJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Adding current status to the buffer
  off_status_buff_.push_front(static_cast<bool>(joy->buttons[off_button_]));
  yellow_status_buff_.push_front(static_cast<bool>(joy->buttons[yellow_button_]));
  green_status_buff_.push_front(static_cast<bool>(joy->buttons[green_button_]));
  blue_status_buff_.push_front(static_cast<bool>(joy->buttons[blue_button_]));


  // Removing last element from the buffer
  off_status_buff_.pop_back();
  yellow_status_buff_.pop_back();
  green_status_buff_.pop_back();
  blue_status_buff_.pop_back();

  if (CheckForRisingEdge(off_status_buff_))
  {
    output_msg_.data = MCU_LED_STATUS_OFF;
    update_output_ = true;
  }
  if (CheckForRisingEdge(yellow_status_buff_))
  {
    output_msg_.data = MCU_LED_STATUS_AUTO_OBST;
    update_output_ = true;
  }
  if (CheckForRisingEdge(green_status_buff_))
  {
    output_msg_.data = MCU_LED_STATUS_MANUAL;
    update_output_ = true;
  }
  if (CheckForRisingEdge(blue_status_buff_))
  {
    output_msg_.data = MCU_LED_STATUS_AUTO_NORMAL;
    update_output_ = true;
  }
}
/**
 * @name  publish
 * @brief The publish timer callback - publishes the data if needed
 */
void AvidbotsLEDJoy::publish()
{
  // Publish outputs if needed
  if (update_output_)
  {
    update_output_   = false;
    led_status_cmd_pub_.publish(output_msg_);
  }
}

/**
 * @name  CheckForRisingEdge
 * @brief Checks for the presence of a raising edge on a buffer
 * @param[in] buffer: The buffer of the current button being analyzed
 */
bool AvidbotsLEDJoy::CheckForRisingEdge(const std::deque<bool> &buffer)
{
  return ((buffer.at(0) == false) && (buffer.at(1) == true)
          && (buffer.at(2) == false));
}
