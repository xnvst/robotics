/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  diagnostics_messenger
 * @brief Header file for DiagnosticsMessenger class
 * @author Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_MESSENGERS_DIAGNOSTICS_MESSENGER_H
#define AVIDBOTS_SAFETY_MONITOR_MESSENGERS_DIAGNOSTICS_MESSENGER_H

// ROS
#include <ros/ros.h>

// CPP
#include <string>

#include <avidbots_msgs/diagnostics.h>

class ROSInterface;

class DiagnosticsMessenger
{
public:
  explicit DiagnosticsMessenger(ROSInterface *ros);

  /* Action Functions */
  void Initialize();
  void StopDiagnosticsTimer();
  void StartDiagnosticsTimer();

  void GetParamSettings();
  void InitializePublishersAndSubscribers();
  void InitializeTimers();

private:
  // Callbacks
  void DiagnosticStatusCallBack(const avidbots_msgs::diagnostics &diagnostics);

  void DiagnosticsTimerCallBack(const ros::TimerEvent&);

  ROSInterface*   ros_;
  ros::NodeHandle node_;
  // Publishers

  // Subscribers
  ros::Subscriber diagnostics_sub_;
  // Timers
  ros::Timer diagnostics_timer_;

  double diagnostics_timeout_;
  bool enable_diagnostics_monitor_;

  ErrorStatus diagnostics_status_,
              mcu_driver_diagnostics_,
              mc_driver_diagnostics_,
              urg_node_diagnostics_,
              camera_left_diagnostics_,
              camera_right_diagnostics_,
              sensor_safety_diagnostics_,
              global_diagnostics_;

  std::string diagnostics_description_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_MESSENGERS_DIAGNOSTICS_MESSENGER_H
