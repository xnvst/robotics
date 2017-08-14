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

#include "avidbots_mc_driver/kiwi_mc_driver.h"

#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_mc_driver/mc_voltage_helper.h"
#include "avidbots_msgs/constants.h"
#include <avidbots_msgs/mc_status.h>
#include <avidbots_msgs/mc_voltage_status.h>
#include <avidbots_msgs/mcu_status.h>
#include <avidbots_msgs/assist_drive_cmd.h>
#include <avidbots_msgs/topics.h>

/**
 * @name  KiwiMCDriver
 * @brief Default constructor
 */
KiwiMCDriver::KiwiMCDriver()
{
  assist_drive_               = false;
  assist_drive_turning_left_  = false;
  assist_drive_turning_right_ = false;

  //  Diagnostics
  connection_status_ = false;
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 * @return  True if the motor controller speed
            commands are not being updated - false otherwise
 */
void KiwiMCDriver::GetParamSettings()
{
  // Retriving motor controller settings
  GetParamUtil::GetParam("/mc_properties/can_port_name", can_port_name_, "can0");
  GetParamUtil::GetParam("/robot_properties/brush_control", brush_control_, "dc");

  if (brush_control_ == "dc")
  {
    GetParamUtil::GetParam("/mc_properties/brush_1_can_id", brush_1_.can_id_, 14);
    GetParamUtil::GetParam("/mc_properties/brush_2_can_id", brush_2_.can_id_, 15);
  }

  GetParamUtil::GetParam("/mc_properties/left_mc_can_id", left_mc_.can_id_, 10);
  GetParamUtil::GetParam("/mc_properties/right_mc_can_id", right_mc_.can_id_, 12);
  GetParamUtil::GetParam("/mc_properties/front_mc_can_id", front_mc_.can_id_, 11);
  GetParamUtil::GetParam("/mc_properties/mc_status_pub_rate", status_pub_rate_, 40);
  GetParamUtil::GetParam("/mc_properties/cmd_timeout", controller_cmd_timeout_, 1);
  GetParamUtil::GetParam("/mc_properties/rpm_to_mc_cmd", conv_rpm_to_mc_cmd_, 0.158);
  GetParamUtil::GetParam("/mc_properties/can_timeout", can_timeout_, 100);

  GetParamUtil::GetParam("/preferences/battery/voltage/min", kMinVoltage, 41.3);
  GetParamUtil::GetParam("/preferences/battery/voltage/max", kMaxVoltage, 53.3);
  GetParamUtil::GetParam("/preferences/battery/voltage/critical", kCriticalVoltage, 42.5);
  kMinVoltage *= 10;
  kMaxVoltage *= 10;
  kCriticalVoltage *= 10;

  GetParamUtil::GetParam("/mc_properties/assist_drive_w_speed",
                         assist_drive_speed_, 0.15);
  GetParamUtil::GetParam("/mc_properties/assist_drive_turning_dir_w_speed",
                         assist_drive_turning_dir_w_speed_, 0.1);
  GetParamUtil::GetParam("/mc_properties/assist_drive_opp_turning_dir_w_speed",
                         assist_drive_opp_turning_dir_w_speed_, 0.2);
  GetParamUtil::GetParam("/robot_properties/invert_right",
                         invert_right_, -1.0);
  GetParamUtil::GetParam("/robot_properties/invert_left",
                         invert_left_, -1.0);
  GetParamUtil::GetParam("/robot_properties/wheel_radius",
                         wheel_radius_, 0.1016);
  GetParamUtil::GetParam("/robot_properties/gear_ratio",
                         gear_ratio_, 44.4);
}

/**
 * @name  InitDriver
 * @brief This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int KiwiMCDriver::InitDriver(ros::NodeHandle *node_handle)
{
  int connect_status = AvidbotsMCCANOpenDriver::InitDriver(node_handle);

  // Set assist drive RPM to each motor
  SetAssistDriveRPM();

  // Start driver thread
  if (connect_status == SocketCANOpen::kCANOpenSuccess)
  {
    driver_thread_ = new boost::thread(
          boost::bind(&KiwiMCDriver::DriverThread, this));
  }
  return connect_status;
}

/**
 * @name  InitMotorObjects
 * @brief Initialize motor and brush objects
 */
void KiwiMCDriver::InitMotorObjects()
{
  front_mc_.Init(can_socket_, true);
  left_mc_.Init(can_socket_, false);
  right_mc_.Init(can_socket_, false);

  if (brush_control_ == "dc")
  {
    brush_1_.Init(can_socket_, false);
    brush_2_.Init(can_socket_, false);
  }
}


/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void KiwiMCDriver::ManagePubAndSub()
{
  // subscribe
  ROS_INFO_STREAM("*** Subscribing for the motor control message ***");
  mc_cmd_sub_       = node_handle_->
                      subscribe(avidbots_topics::mc_cmd, 10,
                      &KiwiMCDriver::MotorCmdCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the assist drive command message ***");
  assist_drive_cmd_sub_ = node_handle_->
                        subscribe(avidbots_topics::mc_assist_drive_cmd, 1,
                        &KiwiMCDriver::AssistDriveCmdCallBack, this);

  AvidbotsMCCANOpenDriver::ManagePubAndSub();
}

/**
 * @name  SetAssistDriveRPM
 * @brief Converts assist drive speed to rpm and set the value to motor objects
 */
void KiwiMCDriver::SetAssistDriveRPM()
{
  /* Convert m/s to rpm for L/R wheels */
  assist_drive_rpm_ = static_cast<int16_t>(
              (assist_drive_speed_/wheel_radius_) * gear_ratio_ * kRadPerSecToRPM)
              * conv_rpm_to_mc_cmd_;

  assist_drive_turning_dir_w_rpm_ = static_cast<int16_t>(
              (assist_drive_turning_dir_w_speed_/wheel_radius_) * gear_ratio_
              * kRadPerSecToRPM) * conv_rpm_to_mc_cmd_;

  assist_drive_opp_turning_dir_w_rpm_ = static_cast<int16_t>(
              (assist_drive_opp_turning_dir_w_speed_/wheel_radius_) * gear_ratio_
              * kRadPerSecToRPM) * conv_rpm_to_mc_cmd_;
}

/**
 * @name  AssistDriveCmdCallBack
 * @brief Callback for assist drive cmd
 * @param[in] msg: The msg containing the assist drive cmd
 */
void KiwiMCDriver::AssistDriveCmdCallBack(
    const avidbots_msgs::assist_drive_cmd &msg)
{
  assist_drive_ = msg.assist_drive;
  assist_drive_turning_left_ = msg.turn_left;
  assist_drive_turning_right_ = msg.turn_right;
}

/**
 * @name    cmdTwistCallBack
 * @brief   Twist command call Back
 * @param[in] vel_cmd: The incoming twist command
 */
void KiwiMCDriver::MotorCmdCallBack(
    const avidbots_msgs::mc_cmdPtr& motor_cmd)
{
  int drive_commands = 0;

  for (int i = 0; i < motor_cmd->commands.size(); i++)
  {
    if (motor_cmd->commands[i].motor == KIWI_WHEEL_F &&
        state_ == MC_DRIVER_STATE_DEFAULT)
    {
      front_mc_.RPM_cmd_ = motor_cmd->commands[i].value * conv_rpm_to_mc_cmd_;
      drive_commands++;
    }
    else if (motor_cmd->commands[i].motor == KIWI_WHEEL_R &&
        state_ == MC_DRIVER_STATE_DEFAULT)
    {
      right_mc_.RPM_cmd_ = motor_cmd->commands[i].value * conv_rpm_to_mc_cmd_;
      drive_commands++;
    }
    else if (motor_cmd->commands[i].motor == KIWI_WHEEL_L &&
        state_ == MC_DRIVER_STATE_DEFAULT)
    {
      left_mc_.RPM_cmd_ = motor_cmd->commands[i].value * conv_rpm_to_mc_cmd_;
      drive_commands++;
    }
    else if (motor_cmd->commands[i].motor == kMotorBrush1)
    {
      brush_1_.RPM_cmd_ = motor_cmd->commands[i].value;
      ROS_DEBUG("Got Brush1 RPM: %d", motor_cmd->commands[i].value);
    }
    else if (motor_cmd->commands[i].motor == kMotorBrush2)
    {
      brush_2_.RPM_cmd_ = motor_cmd->commands[i].value;
      ROS_DEBUG("Got Brush2 RPM: %d", motor_cmd->commands[i].value);
    }
    else
    {
      ROS_WARN("Unknown motor command %d %d", motor_cmd->commands[i].motor,
               motor_cmd->commands[i].value);
    }
  }

  if (drive_commands == 3)  // Todo: When we have different drive systems this should be configurable
  {
    motor_cmd_time_ = can_socket_.GetCurrentTimeStamp();
  }

  // printf("Got RPM command %d\n", left_mc_.RPM_cmd_.load());
}

/**
 * @name    MCUManualMotionCallBack
 * @brief   Manual Motion call Back
 */
void KiwiMCDriver::MCUManualMotionCallBack(const avidbots_msgs::mcu_manualmotionPtr &manualmotion_msg)
{
  // do nothing for now
}

/**
 * @name    UIGearmodeCallBack
 * @brief   UI Gearmode call Back
 */
void KiwiMCDriver::UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode)
{
  // do nothing for now
}

/**
 * @name  PublishStatusCallBack
 * @brief Publish the status of the variables
 */
void KiwiMCDriver::PublishStatusCallBack(const ros::TimerEvent&)
{
  avidbots_msgs::mc_status   status_msg;

  status_msg.amps.push_back(0);

  if (exit_driver_ == false && !exit_driver_loop_)
  {
    status_msg.stamp                  = ros::Time::now();

    status_msg.amps.push_back(0);
    status_msg.amps.push_back(0);
    status_msg.amps.push_back(0);
    status_msg.ticks.push_back(0);
    status_msg.ticks.push_back(0);
    status_msg.ticks.push_back(0);
    status_msg.rpm.push_back(0);
    status_msg.rpm.push_back(0);
    status_msg.rpm.push_back(0);

    status_msg.amps[KIWI_WHEEL_F]     = front_mc_.amps_;
    status_msg.amps[KIWI_WHEEL_R]     = right_mc_.amps_;
    status_msg.amps[KIWI_WHEEL_L]     = left_mc_.amps_;
    status_msg.ticks[KIWI_WHEEL_F]    = front_mc_.ticks_;
    status_msg.ticks[KIWI_WHEEL_R]    = right_mc_.ticks_;
    status_msg.ticks[KIWI_WHEEL_L]    = left_mc_.ticks_;
    status_msg.rpm[KIWI_WHEEL_L]      = left_mc_.RPM_;
    status_msg.rpm[KIWI_WHEEL_R]      = right_mc_.RPM_;
    status_msg.rpm[KIWI_WHEEL_F]      = front_mc_.RPM_;
    status_msg.manual_override        = manual_override_;
    status_msg.assist_drive           = assist_drive_;

    mc_status_pub_.publish(status_msg);
  }
}

/**
 * @name    UpdateDiagnostics
 * @brief   Diagnostics function for kiwi_mc_driver.
 */
void KiwiMCDriver::UpdateDiagnostics(const ros::TimerEvent &)
{
  // do nothing for kiwi driver now
}

/**
 * @name    DriverThread
 * @brief   The main driver thread
 * @return  The result of driver commands -
 *          it will only return if connection/communication has failed
 */
void KiwiMCDriver::DriverThread()
{
  int status_count      = 0;
  uint64_t start        = 0;

  int sequence_max = GetNonCriticalToUpdate();

  if (brush_control_ == "dc")
  {
    sequence_max += brush_1_.non_critical_to_update_ +
        brush_2_.non_critical_to_update_;
  }
  int sequence = 0;

  ROS_INFO_STREAM("*** Starting driver communication thread ***");

  ros::Rate rate(status_pub_rate_);  // 40 Hz

  while (exit_driver_ == false)
  {
    while (exit_driver_loop_ == false && exit_driver_ == false)  // Main driver loop
    {
      start = can_socket_.GetCurrentTimeStamp();

      // State Machine
      switch (state_)
      {
      case MC_DRIVER_STATE_ESTOP:
        if (!estop_state_)
        {
          left_mc_.ticks_   = 0;
          right_mc_.ticks_  = 0;
          front_mc_.ticks_  = 0;
          state_ = MC_DRIVER_STATE_DEFAULT;
          break;
        }
        break;
      case MC_DRIVER_STATE_MANUAL_OVERRIDE:
        if (estop_state_)
        {
          ManualOverride(false);
          state_ = MC_DRIVER_STATE_ESTOP;
          break;
        }
        if (!manual_override_)
        {
          if (assist_drive_)
          {
            ManualOverride(manual_override_);
            AssistDrive(assist_drive_);
            state_ = MC_DRIVER_STATE_ASSIST_DRIVE;
          }
          else
          {
            ManualOverride(manual_override_);
            state_ = MC_DRIVER_STATE_DEFAULT;
          }
          break;
        }
        break;
      case MC_DRIVER_STATE_ASSIST_DRIVE:
        if (estop_state_)
        {
          AssistDrive(false);
          state_ = MC_DRIVER_STATE_ESTOP;
          break;
        }
        if (manual_override_)
        {
          AssistDrive(false);
          ManualOverride(manual_override_);
          state_ = MC_DRIVER_STATE_MANUAL_OVERRIDE;
          break;
        }
        if (!assist_drive_)
        {
          AssistDrive(false);
          state_ = MC_DRIVER_STATE_DEFAULT;
          break;
        }
        AssistDrive(assist_drive_);
        break;
      default:  // case MC_DRIVER_STATE_DEFAULT
        if (estop_state_)
        {
          state_ = MC_DRIVER_STATE_ESTOP;
          break;
        }
        if (manual_override_)
        {
          ManualOverride(manual_override_);
          state_ = MC_DRIVER_STATE_MANUAL_OVERRIDE;
          break;
        }
        if (assist_drive_)
        {
          AssistDrive(assist_drive_);
          state_ = MC_DRIVER_STATE_ASSIST_DRIVE;
          break;
        }
        if (CheckVelocityCommandStatus())
        {
          ROS_DEBUG_STREAM_THROTTLE(
                0.5, "Motor command expired, setting velocities to 0");
          left_mc_.RPM_cmd_   = 0;
          right_mc_.RPM_cmd_  = 0;
          front_mc_.RPM_cmd_  = 0;
        }
        break;
      }
      ROS_DEBUG_THROTTLE(1, "MCState: %d", state_);

      front_mc_.SetRPM();
      left_mc_.SetRPM();
      right_mc_.SetRPM();

      /* The non critical measurements are measrured in a round robin fashion:
       * Each non critical measurement ha s a specific ID relative to its motor position.
       * The sequence variable contains a number from 0 to sequence_max
       * The sequence_so_far keeps track of the specific measurement ID
       * When the sequence number-sequence_so_far equals the non critical
       *   measurement ID, the measurement is taken
       */

      // updating the status global variables and speed
      int sequenceSoFar = 0;
      sequence = (sequence + 1) % sequence_max;

      status_count += front_mc_.RefreshReadings(sequence-sequenceSoFar);
      sequenceSoFar += front_mc_.non_critical_to_update_;
      status_count += left_mc_.RefreshReadings(sequence-sequenceSoFar);
      sequenceSoFar += left_mc_.non_critical_to_update_;
      status_count += right_mc_.RefreshReadings(sequence-sequenceSoFar);
      sequenceSoFar += right_mc_.non_critical_to_update_;

      if (brush_control_ == "dc")
      {
        status_count += brush_1_.RefreshReadings(sequence-sequenceSoFar);
        sequenceSoFar += brush_1_.non_critical_to_update_;
        status_count += brush_2_.RefreshReadings(sequence-sequenceSoFar);
        sequenceSoFar += brush_2_.non_critical_to_update_;
      }

      ROS_DEBUG_STREAM("Amps Motor R: " << right_mc_.amps_ <<
                       " Amp Motor L: " << left_mc_.amps_);
      ROS_DEBUG_STREAM("MC Reading Sequence" << sequence <<
                       ", max:" << sequence_max << " == " << sequenceSoFar);

      if (status_count != 0)
      {
        // there were connection errors in this run
        // disconnect from MC and connect again.
        ROS_DEBUG_STREAM("Driver loop: There were errors: " << status_count);
        exit_driver_loop_ = true;
      }

      // ROS_DEBUG_STREAM("Loop in " << (can_socket_.GetCurrentTimeStamp()-start) << " ms");

      rate.sleep();  // Throttle down to 40Hz
    }

    // Attempt to reconnect if exited because of error
    if (status_count != 0)
    {
      // attempt to reset the CAN socket
      int status = can_socket_.ResetSocket();
      ROS_INFO_STREAM("Reset CAN socket with status code of " << status);


      // Re-enter the loop
      exit_driver_loop_ = false;
      // reset failure counter
      status_count  = 0;
    }
  }
  // Exit thread
  can_socket_.CloseSocket();
}

/**
 * @name AssistDrive
 * @brief Set each motor to be in assisted drive mode if given true.
 * @param assist The boolean status
 */
void KiwiMCDriver::AssistDrive(const bool& assist)
{
  if (assist)
  {
    if (assist_drive_turning_left_ && !assist_drive_turning_right_)
    {
      left_mc_.RPM_cmd_ = -invert_left_ * assist_drive_turning_dir_w_rpm_;
      right_mc_.RPM_cmd_ = invert_right_ * assist_drive_opp_turning_dir_w_rpm_;
    }
    else if (!assist_drive_turning_left_ && assist_drive_turning_right_)
    {
      left_mc_.RPM_cmd_ = -invert_left_ * assist_drive_opp_turning_dir_w_rpm_;
      right_mc_.RPM_cmd_ = invert_right_ * assist_drive_turning_dir_w_rpm_;
    }
    else
    {
      left_mc_.RPM_cmd_ = -invert_left_ * assist_drive_rpm_;
      right_mc_.RPM_cmd_ = invert_right_ * assist_drive_rpm_;
    }
  }
  else
  {
    left_mc_.RPM_cmd_ = 0;
    right_mc_.RPM_cmd_ = 0;
  }

  front_mc_.SetManualOverride(assist);
}

/**
 * @name ManualOverride
 * @brief Set each motor to be in manual override mode if given true.
 * @param override The boolean status
 */
void KiwiMCDriver::ManualOverride(const bool& override)
{
  left_mc_.SetManualOverride(override);
  right_mc_.SetManualOverride(override);
  front_mc_.SetManualOverride(override);
}

/**
 * @name    GetMotorVoltage
 * @brief   Returns front motor controller voltage.
 */
double KiwiMCDriver::GetMotorVoltage()
{
  return front_mc_.voltage_;
}

/**
 * @name    GetNonCriticalToUpdate
 * @brief   Returns number of non-critical update measurements.
 */
int KiwiMCDriver::GetNonCriticalToUpdate()
{
  return left_mc_.non_critical_to_update_ +
      right_mc_.non_critical_to_update_ +
      front_mc_.non_critical_to_update_;
}

/**
 * @name  StartPublishStatusTimer
 * @brief Starts the status publisher timer.  Main loop.
 */
void KiwiMCDriver::StartPublishStatusTimer()
{
  publish_status_timer_ = node_handle_->createTimer
      (ros::Duration(1/status_pub_rate_),
       &KiwiMCDriver::PublishStatusCallBack, this);

  ros::Rate rate(status_pub_rate_);
  while (!exit_driver_)
  {
    rate.sleep();
    ros::spinOnce();
  }
}
