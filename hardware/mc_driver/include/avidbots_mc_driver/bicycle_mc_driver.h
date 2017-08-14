/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  bicycle_mc_driver.h
 * @brief Header File containing the MC driver class for bicycle drive system
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MC_DRIVER_BICYCLE_MC_DRIVER_H
#define AVIDBOTS_MC_DRIVER_BICYCLE_MC_DRIVER_H

#include "avidbots_msgs/mc_sq_pos_cmd.h"
#include "avidbots_msgs/mc_ch_pos_cmd.h"
#include "avidbots_msgs/back_wheel_encoder_status.h"
#include "avidbots_msgs/mc_states.h"
#include "avidbots_mc_driver/avidbots_mc_driver.h"
#include "avidbots_mc_driver/roboteq_ch_position_motor.h"
#include "avidbots_mc_driver/roboteq_sq_position_motor.h"
#include "avidbots_mc_driver/roboteq_drive_system_motor.h"
#include "avidbots_mc_driver/roboteq_steer_system_motor.h"

class BicycleMCDriver : public AvidbotsMCCANOpenDriver
{
private:
  ros::Subscriber ch_pos_sub_,
                  sq_pos_sub_;

  // Motor controller settings
  RoboteqDriveSystemMotor wheel_mc_;
  RoboteqSteerSystemMotor steering_mc_;
  RoboteqCHPositionMotor  ch_pos_mc_;
  RoboteqSQPositionMotor  sq_pos_mc_;

  // gearmode relayed by MCU from UI
  uint8_t gearmode_control_;
  uint8_t old_gearmode_control_;
  // gearmode from UI
  uint8_t ui_gearmode_control_;
  uint8_t old_ui_gearmode_control_;

  bool mc_sq_pos_control_;
  bool mc_ch_pos_control_;
  bool reset_ticks_;

  int sequence_max_,
      sequence_,
      status_count_,
      ch_pos_update_sequence_;

  bool initialized_;

  ros::Publisher calib_status_pub_;
  ros::Publisher back_wheel_encoder_pub_;
  ros::Publisher mc_states_pub_;

  void CheckMeasurements();
  void PublishCalibrationStatus(const bool& status);

  // Motor controller driver states specific for the Bicycle MC Driver.
  static const int MC_DRIVER_STATE_CALIBRATE = 3;

protected:
  // Initialization helper functions
  void GetParamSettings();
  void InitMotorObjects();
  void ManagePubAndSub();

  // Callbacks
  void SQPosCmdCallBack(const avidbots_msgs::mc_sq_pos_cmd& sq_pos);
  void CHPosCmdCallBack(const avidbots_msgs::mc_ch_pos_cmd& ch_pos);
  void MotorCmdCallBack(const avidbots_msgs::mc_cmdPtr &motor_cmd);
  void MCUManualMotionCallBack(const avidbots_msgs::mcu_manualmotionPtr &manualmotion_msg);
  void UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode);
  void PublishStatusCallBack(const ros::TimerEvent &);

  // Core functionality behaviours
  void DriverThread();
  void UpdateDiagnostics(const ros::TimerEvent &);

  // Helper functions
  double GetMotorVoltage();

public:
  BicycleMCDriver();

  // Initialization functions
  int InitDriver(ros::NodeHandle *node_handle);
  void StartPublishStatusTimer();
};

#endif  // AVIDBOTS_MC_DRIVER_BICYCLE_MC_DRIVER_H
