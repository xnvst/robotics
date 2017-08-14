/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  kiwi_mc_driver.h
 * @brief Header File containing the mc driver class for kiwi drive system
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MC_DRIVER_KIWI_MC_DRIVER_H
#define AVIDBOTS_MC_DRIVER_KIWI_MC_DRIVER_H

// C++
#include <string>

// External
#include <avidbots_msgs/mc_cmd.h>
#include <avidbots_msgs/mc_status.h>
#include <avidbots_msgs/mcu_status.h>
#include <avidbots_msgs/assist_drive_cmd.h>

// Local
#include "avidbots_mc_driver/avidbots_mc_driver.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can.h"
#include "avidbots_mc_driver/roboteq_drive_system_motor.h"
#include "avidbots_mc_driver/roboteq_cleaning_head_motor.h"

class KiwiMCDriver : public AvidbotsMCCANOpenDriver
{
private:
  // Motors
  RoboteqDriveSystemMotor front_mc_;
  RoboteqDriveSystemMotor left_mc_;
  RoboteqDriveSystemMotor right_mc_;

  RoboteqCleaningHeadMotor brush_1_;
  RoboteqCleaningHeadMotor brush_2_;

  // Motor parameters
  std::string brush_control_;
  double invert_right_;
  double invert_left_;

  // Assist Drive Speed
  const double kRadPerSecToRPM = 9.54929659643;
  double assist_drive_speed_;
  double assist_drive_turning_dir_w_speed_;
  double assist_drive_opp_turning_dir_w_speed_;

  // Motor controller driver states specific for the Kiwi MC Driver.
  static const int MC_DRIVER_STATE_ASSIST_DRIVE = 3;

  // State flag
  int16_t assist_drive_rpm_;
  int16_t assist_drive_turning_dir_w_rpm_;
  int16_t assist_drive_opp_turning_dir_w_rpm_;

  bool assist_drive_;
  bool assist_drive_turning_left_;
  bool assist_drive_turning_right_;

  // ROS publisher/subscriber
  ros::Subscriber assist_drive_cmd_sub_;

protected:
  // Initailization helper functions
  void GetParamSettings();
  void InitMotorObjects();
  void ManagePubAndSub();
  void SetAssistDriveRPM();  // Extended class behaviour

  // Callbacks
  void AssistDriveCmdCallBack(const avidbots_msgs::assist_drive_cmd& msg);  // Extended class behaviour
  void MotorCmdCallBack(const avidbots_msgs::mc_cmdPtr &motor_cmd);
  void MCUManualMotionCallBack(const avidbots_msgs::mcu_manualmotionPtr &manualmotion_msg);
  void UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode);
  void PublishStatusCallBack(const ros::TimerEvent&);

  // Core functionality behaviours
  void DriverThread();
  void UpdateDiagnostics(const ros::TimerEvent &);

  // Helper functions
  void AssistDrive(const bool& assist);  // Extended class behaviour
  void ManualOverride(const bool& override);  // Extended class behaviour
  double GetMotorVoltage();
  int GetNonCriticalToUpdate();  // Extended class behaviour

public:
  KiwiMCDriver();

  // Initialization functions
  int InitDriver(ros::NodeHandle *node_handle);
  void StartPublishStatusTimer();
};

#endif  // AVIDBOTS_MC_DRIVER_KIWI_MC_DRIVER_H
