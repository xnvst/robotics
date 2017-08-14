/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    ros_interface
 * @brief   Header file for ROSInterface class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SM_ROS_INTERFACE_H
#define AVIDBOTS_SAFETY_MONITOR_SM_ROS_INTERFACE_H


// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/sm_manager.h"
#include "avidbots_library/safety_zone/properties.h"
#include "avidbots_safety_monitor/sm_constants.h"
#include "avidbots_safety_monitor/messengers/diagnostics_messenger.h"
#include "avidbots_safety_monitor/messengers/low_voltage_messenger.h"
#include "avidbots_safety_monitor/messengers/safetyzone_messenger.h"

// MSGS
#include <avidbots_msgs/topics.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <avidbots_msgs/ui_state.h>
#include <avidbots_msgs/mcu_status.h>
#include <avidbots_msgs/sm_status.h>
#include <avidbots_msgs/sensor_toggle.h>
#include <avidbots_msgs/mc_states.h>

class SMManager;

/**
 * @name    ROSInterface
 * @brief   Handles communication with ROS topics.
 */
class ROSInterface
{
  friend class DiagnosticsMessenger;
  friend class DangerZoneMessenger;
  friend class LowVoltageMessenger;
public:
  explicit ROSInterface(SMManager* manager);
  ~ROSInterface();

  /* UI Manager Object */
  SMManager*      manager_;

  void Initialize();

  /* Publish Functions */
  void PublishStateMsg(std_msgs::String state_msg_);
  void PublishErrorMsg(std_msgs::String error_msg);
  void PublishSMStatus(avidbots_msgs::sm_status sm_status);
  void PublishZeroVelocity();
  void PublishDangerZoneObstacle(bool status);
  void PublishMCUOutputCommand();
  void PublishMCUOutputCommand(const int &value);
  void PublishSensorSafetyToggle(const bool &value);

private:
  ros::NodeHandle             node_;

  // Publishers
  ros::Publisher vel_muxer_pub_,
                 sm_state_pub_,
                 sm_status_pub_,
                 sm_error_pub_,
                 succ_fail_handshake_pub_,
                 mcu_output_pub_,
                 sensor_safety_pub_;

  // Subscribers
  ros::Subscriber sm_failure_reset_sub_,
                          mcu_status_sub_,
                          mc_states_sub_,
                          ui_state_sub_;

  // Timers
  ros::Timer monitor_timer_,
             mcu_handshake_timer_;
  ros::Time node_start_time_;
  ros::Time diagnostics_start_time_;

  // Create Messenger object pointers here
  DiagnosticsMessenger* diagnostics_messenger_;
  LowVoltageMessenger* low_voltage_messenger_;
  SafetyZoneMessenger* safetyzone_messenger_;

  /* Callbacks */
  void MCUStatusCallBack(const avidbots_msgs::mcu_status& mcu_status_msg);
  void MCStatesCallBack(const avidbots_msgs::mc_states& mc_states_msg);
  void SmFailureResetCallBack(const std_msgs::Bool& sm_failure_reset_msg);
  void MainMonitorTimerCallBack(const ros::TimerEvent &);
  void MCUHandshakeTimerCallBack(const ros::TimerEvent &);
  void UIStateCallback(const avidbots_msgs::ui_state& ui_state_msg);

  /* Helper Function */
  void GetParamSettings();
  void InitializePublishersAndSubscribers();
  void InitializeTimers();

  bool enable_safety_monitor_;
  bool sm_failure_reset_;
  int mcu_state_;
  int mc_state_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_SM_ROS_INTERFACE_H
