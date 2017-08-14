/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  low_voltage_messenger
 * @brief Header file for LowVoltageMessenger class
 * @author Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_MESSENGERS_LOW_VOLTAGE_MESSENGER_H
#define AVIDBOTS_SAFETY_MONITOR_MESSENGERS_LOW_VOLTAGE_MESSENGER_H

// ROS
#include <ros/ros.h>

#include <avidbots_msgs/mc_voltage_status.h>

class ROSInterface;

class LowVoltageMessenger
{
public:
  explicit LowVoltageMessenger(ROSInterface *ros);

  /* Action Functions */
  void Initialize();

  /* Helper Functions */
  void GetParamSettings();
  void InitializePublishersAndSubscribers();
  void InitializeTimers();

private:
  // Callbacks
  void MCVoltageStatusCallBack(const avidbots_msgs::mc_voltage_statusPtr&
                               voltage_status_msg);
  void LowVoltageTimerCallBack(const ros::TimerEvent &);

  ROSInterface*   ros_;
  ros::NodeHandle node_;
  // Publishers

  // Subscribers
  ros::Subscriber mc_voltage_status_sub_;

  // Timers
  ros::Timer voltage_timer_;

  bool enable_low_voltage_monitor_,
       voltage_is_valid_;
  int voltage_low_counter_;
  double kMinVoltage_,
         kCriticalVoltage_,
         current_voltage_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_MESSENGERS_LOW_VOLTAGE_MESSENGER_H
