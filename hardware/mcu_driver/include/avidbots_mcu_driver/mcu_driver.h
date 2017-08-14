/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	avidbots_mcu_driver.cpp
 * @brief	Header file containing the Avidbots MCU driver class
 * @author	Pablo Molina
 */

#ifndef AVIDBOTS_MCU_DRIVER_MCU_DRIVER_H
#define AVIDBOTS_MCU_DRIVER_MCU_DRIVER_H


#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/mcu_output_cmd.h>
#include <avidbots_msgs/mcu_manualmotion.h>
#include <avidbots_msgs/gearmode.h>
#include <avidbots_msgs/mcu_firmware_version.h>
#include <avidbots_msgs/diagnostics_states.h>
#include "avidbots_mcu_driver/avidbots_can_open.h"
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"

class AvidbotsMCUDriver
{
protected:
  // Local variables
  uint8_t mcu_state_variable_;
  uint8_t mcu_manual_override_;
  uint8_t mcu_estop_status_;
  bool mcu_estop_state_;
  bool mcu_warning_off_state_;
  uint8_t gearmode_state_;

  // diagnostics_variables
  bool connection_status_;

  // Settings
  std::string can_port_name_;
  int  can_timeout_;
  int  mcu_can_id_;

  // Control variables
  bool exit_driver_;
  bool exit_driver_loop_;
  int driver_loop_rate_;

  // ROS publisher/subscriber
  ros::NodeHandle*  node_handle_;
  ros::Publisher    mcu_status_pub_,
                    mcu_estop_status_pub_,
                    mcu_warning_off_pub_,
                    mcu_disconnection_pub_,
                    mcu_dropped_msg_pub_,
                    mcu_manualmotion_pub_,
                    mcu_firmware_version_pub_,
                    mcu_diagnostics_states_pub_;
  ros::Subscriber   mcu_output_cmd_sub_,
                    ui_gearmode_sub_;

  AvidbotsCANOpenController can_controller_;

  // Helpers
  int ConnectToCanSocket(const std::string &port);

  virtual void ManagePubAndSub();

  virtual void GetParamSettings(const ros::NodeHandle& private_nh);

  void PublishMCUStateUpdate(const uint8_t& mcu_state_var);

  // CallBacks
  void OutputCmdCallBack(const avidbots_msgs::mcu_output_cmdPtr &mcu_output_cmd);
  void UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode);

public:
  virtual ~AvidbotsMCUDriver();
  AvidbotsMCUDriver();

  virtual int InitDriver(const ros::NodeHandle& private_nh,
                         ros::NodeHandle* node_handle);

  virtual void StartPublishStatusLoop() = 0;

  // Diagnostics
  ros::Timer diagnostics_timer_;
  void UpdateDiagnostics(const ros::TimerEvent &);
};

#endif  // AVIDBOTS_MCU_DRIVER_MCU_DRIVER_H
