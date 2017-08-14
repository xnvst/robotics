/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  avidbots_mc_driver.h
 * @brief Header file that defines the common behaviours of all MC driver classes
 * @author  Pablo Molina, Jake Park
 */

#ifndef AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_H
#define AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <avidbots_msgs/mc_cmd.h>
#include <avidbots_msgs/mc_status.h>
#include <avidbots_msgs/mcu_status.h>
#include <avidbots_msgs/mcu_manualmotion.h>
#include <avidbots_msgs/gearmode.h>
#include <avidbots_msgs/diagnostics_states.h>
#include "avidbots_library/socket_can/socket_can_open.h"
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"

// BOOST
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/circular_buffer.hpp>

class AvidbotsMCCANOpenDriver
{
protected:
  SocketCANOpenProtocol can_socket_;

  std::atomic<uint64_t> motor_cmd_time_;

  double conv_rpm_to_mc_cmd_;

  // Diagnostics variables
  bool connection_status_;
  double kMinVoltage;
  double kMaxVoltage;
  double kCriticalVoltage;

  bool voltage_low_;

  int kManualOverrideTimeout = 500;  // milliseconds

  // Voltage Data
  int kVoltageBufferSize = 256;
  boost::circular_buffer<int> voltage_data_;
  double motor_voltage_;

  // Settings
  std::string can_port_name_;
  int controller_cmd_timeout_;
  int can_timeout_;

  double wheel_radius_;
  double gear_ratio_;

  // Control variables
  bool exit_driver_;
  bool exit_driver_loop_;
  int status_pub_rate_;
  int voltage_pub_rate_;

  // Motor controller driver states for the Avidbots MC Driver.
  static const int MC_DRIVER_STATE_DEFAULT = 0;
  static const int MC_DRIVER_STATE_ESTOP = 1;
  static const int MC_DRIVER_STATE_MANUAL_OVERRIDE = 2;
  int state_;

  // State flag
  bool estop_state_;
  bool manual_override_;

  // ROS publisher/subscriber
  ros::NodeHandle* node_handle_;
  ros::Publisher mc_status_pub_;
  ros::Publisher mc_voltage_pub_;
  ros::Publisher mc_diagnostics_states_pub_;

  ros::Subscriber mc_cmd_sub_;
  ros::Subscriber manual_override_sub_;
  ros::Subscriber mcu_estop_cmd_sub_;
  ros::Subscriber mcu_manualmotion_sub_;
  ros::Subscriber ui_gearmode_sub_;

  ros::Timer publish_status_timer_;
  ros::Timer publish_voltage_status_timer_;

  // Thread related
  boost::thread* driver_thread_;
  virtual void DriverThread() = 0;

  // Diagnostics
  ros::Timer diagnostics_timer_;
  virtual void UpdateDiagnostics(const ros::TimerEvent &) = 0;

  // Helper functions
  virtual double GetMotorVoltage() = 0;

  // CallBacks
  void ManualOverrideCallBack(const std_msgs::BoolPtr& manual_override_msg);
  void MCUEstopCallBack(const std_msgs::Bool &msg);
  virtual void MotorCmdCallBack(const avidbots_msgs::mc_cmdPtr &motor_cmd) = 0;
  virtual void PublishStatusCallBack(const ros::TimerEvent&) = 0;
  void PublishVoltageCallBack(const ros::TimerEvent&);
  virtual void MCUManualMotionCallBack(const avidbots_msgs::mcu_manualmotionPtr &manualmotion_msg) = 0;
  virtual void UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode) = 0;

  // Initialization helpers
  bool CheckVelocityCommandStatus();
  int ConnectToCanSocket(const std::string &port);
  virtual void GetParamSettings() = 0;
  virtual void InitMotorObjects() = 0;
  void InitTimers();
  virtual void ManagePubAndSub();

public:
  virtual ~AvidbotsMCCANOpenDriver();
  AvidbotsMCCANOpenDriver();

  // Driver Initialization function
  virtual int InitDriver(ros::NodeHandle* node_handle);

  // Status message publisher initializer
  virtual void StartPublishStatusTimer() = 0;

  // Command function to exit driver thread
  void ExitDriverThread();
};

#endif  // AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_H
