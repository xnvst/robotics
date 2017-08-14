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
 * @brief Source file containing the MC driver class for bicycle drive system
 * @author  Jake Park
 */

// ROS
#include <ros/ros.h>

// CPP
#include <string>

// Local
#include "avidbots_library/socket_can/avidbots_can_open_constants.h"
#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"
#include "avidbots_mc_driver/roboteq_ch_position_motor.h"
#include "avidbots_mc_driver/roboteq_sq_position_motor.h"
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_mc_driver/mc_voltage_helper.h"
#include "avidbots_mc_driver/bicycle_mc_driver.h"
#include "avidbots_msgs/mc_sq_pos_cmd.h"
#include "avidbots_msgs/mc_ch_pos_cmd.h"
#include "avidbots_msgs/constants.h"
#include <avidbots_msgs/topics.h>


/**
 * @name  BicycleMCDriver
 * @brief Default constructor
 */
BicycleMCDriver::BicycleMCDriver()
{
  connection_status_ = false;

  // Initialize sequence/status_count to zero
  sequence_ = status_count_ = ch_pos_update_sequence_ = 0;

  // Initialize state to MC_DRIVER_STATE_CALIBRATE
  state_ = MC_DRIVER_STATE_CALIBRATE;
  initialized_ = false;
  reset_ticks_ = false;
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 * @return  True if the motor controller speed
            commands are not being updated - false otherwise
 */
void BicycleMCDriver::GetParamSettings()
{
  // Retriving motor controller settings
  GetParamUtil::GetParam("/mc_properties/can_port_name", can_port_name_, "can0");

  // CAN ID
  GetParamUtil::GetParam("/mc_properties/wheel_mc_can_id", wheel_mc_.can_id_, 10);
  GetParamUtil::GetParam("/mc_properties/steering_mc_can_id", steering_mc_.can_id_, 11);
  GetParamUtil::GetParam("/mc_properties/ch_pos_motor_can_id",  ch_pos_mc_.can_id_, 12);
  GetParamUtil::GetParam("/mc_properties/sq_pos_motor_can_id",  sq_pos_mc_.can_id_, 12);

  // Status publisher rate
  GetParamUtil::GetParam("/mc_properties/mc_status_pub_rate", status_pub_rate_, 40);

  // CAN and controller timeout values
  GetParamUtil::GetParam("/mc_properties/cmd_timeout", controller_cmd_timeout_, 1);
  GetParamUtil::GetParam("/mc_properties/can_timeout", can_timeout_, 100);

  // RPM to MC CMD conversion
  GetParamUtil::GetParam("/mc_properties/rpm_to_mc_cmd", conv_rpm_to_mc_cmd_, 0.3125);

  // CHECK FOR THE RIGHT MIN AND MAX VOLTAGE
  GetParamUtil::GetParam("/preferences/battery/voltage/min", kMinVoltage, 32.0);
  GetParamUtil::GetParam("/preferences/battery/voltage/max", kMaxVoltage, 39.6);
  GetParamUtil::GetParam("/preferences/battery/voltage/critical", kCriticalVoltage, 32.76);
  kMinVoltage *= 10;
  kMaxVoltage *= 10;
  kCriticalVoltage *= 10;

  // Wheel parameters
  GetParamUtil::GetParam("/robot_properties/wheel_radius",
                         wheel_radius_, 0.1);
  GetParamUtil::GetParam("/robot_properties/gear_ratio",
                         gear_ratio_, 27.271);

  // Check if MC driver is used to control cleaning head and squeegee
  std::string ch_pos_control, sq_pos_control;
  GetParamUtil::GetParam("/robot_properties/ch_pos_control", ch_pos_control, USE_MC);
  GetParamUtil::GetParam("/robot_properties/sq_pos_control", sq_pos_control, USE_MC);
  mc_ch_pos_control_ = (ch_pos_control == USE_MC);
  mc_sq_pos_control_ = (sq_pos_control == USE_MC);
}

/**
 * @name  InitMotorObjects
 * @brief Initialize motor and brush objects
 */
void BicycleMCDriver::InitMotorObjects()
{
  wheel_mc_.Init(can_socket_, false);
  steering_mc_.Init(can_socket_, true);

  sequence_max_ = wheel_mc_.non_critical_to_update_ +
                  steering_mc_.non_critical_to_update_;

  if (mc_sq_pos_control_)
  {
    sq_pos_mc_.Init(can_socket_, false);
    sequence_max_ += sq_pos_mc_.non_critical_to_update_;
  }
  if (mc_ch_pos_control_)
  {
    /* NOTE: CH Position control is NOT part of the main round robin sequence */
    ch_pos_mc_.Init(can_socket_, false);
  }
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void BicycleMCDriver::ManagePubAndSub()
{
  // subscribe
  ROS_INFO_STREAM("*** Subscribing for the motor control message ***");
  mc_cmd_sub_       = node_handle_->
                      subscribe(avidbots_topics::mc_cmd, 10,
                      &BicycleMCDriver::MotorCmdCallBack, this);
  if (mc_sq_pos_control_)
  {
    ROS_INFO("*** Subscribing for squeegee position message ***");
    sq_pos_sub_     = node_handle_->subscribe(avidbots_topics::mc_sq_pos, 10,
                                              &BicycleMCDriver::SQPosCmdCallBack, this);
  }
  if (mc_ch_pos_control_)
  {
    ROS_INFO("*** Subscribing for cleaning head position message ***");
    ch_pos_sub_     = node_handle_->subscribe(avidbots_topics::mc_ch_pos, 10,
                                              &BicycleMCDriver::CHPosCmdCallBack, this);
  }

  ROS_INFO_STREAM("*** Subscribing for the MCU Manual Motion message ***");
  mcu_manualmotion_sub_    = node_handle_->
                      subscribe(avidbots_topics::mcu_manualmotion_topic, 1,
                      &BicycleMCDriver::MCUManualMotionCallBack, this);

  ROS_INFO_STREAM("*** Subscribing for the UI gearmode message from MC ***");
  ui_gearmode_sub_    = node_handle_->
                      subscribe(avidbots_topics::ui_gearmode_topic, 10,
                      &BicycleMCDriver::UIGearmodeCallBack, this);

  // publish
  ROS_INFO_STREAM("*** Publishing the calibration status message ***");
  calib_status_pub_ = node_handle_->advertise<std_msgs::Bool>
                      (avidbots_topics::mc_steering_calibration_topic, 10, true);

  if (mc_sq_pos_control_ && mc_ch_pos_control_)
  {
    ROS_INFO("*** Publishing back wheel enconder ticks ***");
    back_wheel_encoder_pub_ = node_handle_->advertise<avidbots_msgs::back_wheel_encoder_status>
                                            (avidbots_topics::mc_back_wheel_encoder_status, 10, true);
  }

  ROS_INFO_STREAM("*** Publishing MC state message ***");
  mc_states_pub_ = node_handle_->advertise<avidbots_msgs::mc_states>
                      (avidbots_topics::mc_states_topic, 10, true);

  AvidbotsMCCANOpenDriver::ManagePubAndSub();
}


/**
 * @name       SQPosCmdCallBack
 * @brief      squeegee position command callback
 * @param[in]  sq_pos: squeegee position
 */
void BicycleMCDriver::SQPosCmdCallBack(const avidbots_msgs::mc_sq_pos_cmd& sq_pos)
{
  ROS_DEBUG("[BicycleMCDriver] SQ position command received: %d", sq_pos.position);
  sq_pos_mc_.RPM_cmd_ = sq_pos.position;
}

/**
 * @name       CHPosCmdCallBack
 * @brief      cleaning head position command callBack
 * @param[in]  ch_pos: cleaning head position
 */
void BicycleMCDriver::CHPosCmdCallBack(const avidbots_msgs::mc_ch_pos_cmd& ch_pos)
{
  ch_pos_mc_.RPM_cmd_ = ch_pos.position;
}


/**
 * @name    cmdTwistCallBack
 * @brief   Twist command call Back
 * @param[in] vel_cmd: The incoming twist command
 */
void BicycleMCDriver::MotorCmdCallBack(const avidbots_msgs::mc_cmdPtr &motor_cmd)
{
  if (state_ == MC_DRIVER_STATE_DEFAULT)
  {
    for (int i = 0; i < motor_cmd->commands.size(); i++)
    {
      if (motor_cmd->commands[i].motor == BICYCLE_WHEEL_DRIVE)
      {
        wheel_mc_.RPM_cmd_ = motor_cmd->commands[i].value * conv_rpm_to_mc_cmd_;
      }
      else if (motor_cmd->commands[i].motor == BICYCLE_STEERING_CONTROL)
      {
        steering_mc_.RPM_cmd_ = motor_cmd->commands[i].value;
      }
      else
      {
        ROS_WARN("Unknown motor command %d %d", motor_cmd->commands[i].motor,
                 motor_cmd->commands[i].value);
      }
    }

    motor_cmd_time_ = can_socket_.GetCurrentTimeStamp();
  }
}

/**
 * @name    MCUManualMotionCallBack
 * @brief   Manual Motion call Back
 */
void BicycleMCDriver::MCUManualMotionCallBack(const avidbots_msgs::mcu_manualmotionPtr &manualmotion_msg)
{
  gearmode_control_ = manualmotion_msg->gearmode_val;
  ROS_DEBUG("MCUManualMotionCallBack: gearmode=0x%x\n", manualmotion_msg->gearmode_val);
  if (state_ == MC_DRIVER_STATE_MANUAL_OVERRIDE) {
    ROS_DEBUG("MCUManualMotionCallBack: steering=0x%x, driving=0x%x\n",   manualmotion_msg->steering_val,
                                                                          manualmotion_msg->driving_val);
    steering_mc_.RPM_cmd_ = manualmotion_msg->steering_val;
    wheel_mc_.RPM_cmd_ = manualmotion_msg->driving_val;
  }
}

/**
 * @name    UIGearmodeCallBack
 * @brief   UI Gearmode call Back
 */
void BicycleMCDriver::UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode)
{
  if (ui_gearmode_control_ != gearmode->value)
  {
    old_ui_gearmode_control_ = ui_gearmode_control_;
    ui_gearmode_control_ = gearmode->value;
    if (ui_gearmode_control_ == avidbots_msgs::gearmode::kParkingMode)
    {
      wheel_mc_.SetShutdown(0);
      steering_mc_.SetShutdown(0);
    }
    else if (ui_gearmode_control_ == avidbots_msgs::gearmode::kNeutralMode)
    {
      wheel_mc_.SetShutdown(1);
      steering_mc_.SetShutdown(1);
    }
    ROS_DEBUG("UIGearmodeCallBack: ui_gearmode=0x%x, 0x%x\n", old_ui_gearmode_control_, ui_gearmode_control_);
  }
}

/**
 * @name  PublishStatusCallBack
 * @brief Publish the status of the variables
 */
void BicycleMCDriver::PublishStatusCallBack(const ros::TimerEvent&)
{
  avidbots_msgs::mc_status   status_msg;

  status_msg.amps.push_back(0);

  if (exit_driver_ == false && !exit_driver_loop_ &&
      state_ != MC_DRIVER_STATE_CALIBRATE &&
      state_ != MC_DRIVER_STATE_ESTOP)
  {
    if (reset_ticks_)
    {
      // Manually make sure the ticks are resetted before updating them by sleeping for 0.5s
      ROS_DEBUG("Sleeping for 0.5s to wait until tick values stablize.");
      ros::Duration(0.5).sleep();
      reset_ticks_ = false;
    }
    else
    {
      status_msg.stamp                  = ros::Time::now();

      status_msg.amps.push_back(0);
      status_msg.amps.push_back(0);
      status_msg.ticks.push_back(0);
      status_msg.ticks.push_back(0);
      status_msg.rpm.push_back(0);
      status_msg.rpm.push_back(0);

      status_msg.amps[BICYCLE_WHEEL_DRIVE]       = wheel_mc_.amps_;
      status_msg.amps[BICYCLE_STEERING_CONTROL]  = steering_mc_.amps_;
      status_msg.ticks[BICYCLE_WHEEL_DRIVE]      = wheel_mc_.ticks_;
      status_msg.ticks[BICYCLE_STEERING_CONTROL] = steering_mc_.ticks_;
      status_msg.rpm[BICYCLE_WHEEL_DRIVE]        = wheel_mc_.RPM_;
      status_msg.rpm[BICYCLE_STEERING_CONTROL]   = steering_mc_.RPM_;
      status_msg.manual_override                 = manual_override_;

      // *** HACK: By necessity, left rear motor ticks are read by the cleaning head control motor and
      //           right rear motor ticks are read by the right squeege position control motor
      if (mc_sq_pos_control_ && mc_ch_pos_control_)
      {
        avidbots_msgs::back_wheel_encoder_status back_wheel_status_msg;

        back_wheel_status_msg.stamp = ros::Time::now();

        back_wheel_status_msg.left_back_wheel_encoder_ticks  = ch_pos_mc_.ticks_;
        back_wheel_status_msg.right_back_wheel_encoder_ticks = sq_pos_mc_.ticks_;
        back_wheel_encoder_pub_.publish(back_wheel_status_msg);
      }

      mc_status_pub_.publish(status_msg);
    }
  }
}

/**
 * @name    UpdateDiagnostics
 * @brief   Diagnostics function for bicycle_mc_driver.
 */
void BicycleMCDriver::UpdateDiagnostics(const ros::TimerEvent &)
{
  ROS_DEBUG_STREAM("BicycleMCDriver::UpdateDiagnostics\n");

  //  publish mc states
  avidbots_msgs::mc_states mc_states_msg;
  mc_states_msg.stamp = ros::Time::now();
  mc_states_msg.state = state_;
  mc_states_pub_.publish(mc_states_msg);

  // publish mc diagnostics states
  avidbots_msgs::diagnostics_states mc_diagnostics_states_msg;
  mc_diagnostics_states_msg.stamp = ros::Time::now();
  mc_diagnostics_states_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCDriver;
  mc_diagnostics_states_msg.description = "";
  if (wheel_mc_.error_state_ || steering_mc_.error_state_ || exit_driver_loop_)
  {
    mc_diagnostics_states_msg.states = avidbots_msgs::diagnostics_states::kStateError;
    if (wheel_mc_.error_state_)
    {
      mc_diagnostics_states_msg.description += "MCwheel.";
    }
    if (steering_mc_.error_state_)
    {
      mc_diagnostics_states_msg.description += "MCsteer.";
    }
    if (exit_driver_loop_)
    {
      mc_diagnostics_states_msg.description += "MCexit.";
    }
  }
  else
  {
    mc_diagnostics_states_msg.states = avidbots_msgs::diagnostics_states::kStateOk;
  }
  mc_diagnostics_states_pub_.publish(mc_diagnostics_states_msg);
}

/**
 * @name    DriverThread
 * @brief   The main driver thread
 * @return  The result of driver commands -
 *          it will only return if connection/communication has failed
 */
void BicycleMCDriver::DriverThread()
{
  uint64_t start        = 0;

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
        if (!estop_state_ && steering_mc_.calibration_)
        {
          ROS_WARN("MCDriver: Switching from ESTOP to CALIBRATE");
          state_ = MC_DRIVER_STATE_CALIBRATE;
          PublishCalibrationStatus(true);
          break;
        }
        break;
      case MC_DRIVER_STATE_CALIBRATE:
        if (estop_state_)
        {
          ROS_WARN("MCDriver: Switching from CALIBRATE to ESTOP");
          state_ = MC_DRIVER_STATE_ESTOP;
          PublishCalibrationStatus(false);
          break;
        }
        /* When the motor controller starts up, calibration doesn't start
         * right away. initialized_ flag has been added to remain in
         * STATE_CALIBRATE until calibration actually happens during initialization.
         */
        if (!initialized_)
        {
          initialized_ = steering_mc_.calibration_;
          PublishCalibrationStatus(true);
          break;
        }

        if (!steering_mc_.calibration_ && initialized_)
        {
          ROS_WARN("MCDriver: Switching from CALIBRATE to DEFAULT");
          wheel_mc_.ticks_ = 0;
          steering_mc_.ticks_ = 0;
          state_ = MC_DRIVER_STATE_DEFAULT;
          PublishCalibrationStatus(false);
          reset_ticks_ = true;
          break;
        }
        break;
      case MC_DRIVER_STATE_MANUAL_OVERRIDE:
        // Parking mode control - from UI directly
        if (ui_gearmode_control_ == avidbots_msgs::gearmode::kParkingMode)
        {
          wheel_mc_.SetShutdown(0);
          steering_mc_.SetShutdown(0);
        }
        if (estop_state_)
        {
          ROS_WARN("MCDriver: Switching from MANOVRD to ESTOP");
          state_ = MC_DRIVER_STATE_ESTOP;
          break;
        }
        if (!manual_override_)
        {
          ROS_WARN("MCDriver: Switching from MANOVRD to DEFAULT");
          // Parking mode control - redundancy check protection for N to P
          if (gearmode_control_ == avidbots_msgs::gearmode::kNeutralMode)
          {
            wheel_mc_.SetShutdown(0);
            steering_mc_.SetShutdown(0);
          }
          state_ = MC_DRIVER_STATE_DEFAULT;
          break;
        }
        if (old_gearmode_control_ != gearmode_control_)
        {
          if (gearmode_control_ == avidbots_msgs::gearmode::kNeutralMode)
          {
            wheel_mc_.SetShutdown(1);
            steering_mc_.SetShutdown(1);
          }
          else if (old_gearmode_control_ == avidbots_msgs::gearmode::kNeutralMode)
          {
            wheel_mc_.SetShutdown(0);
            steering_mc_.SetShutdown(0);
          }
          old_gearmode_control_ = gearmode_control_;
        }
        wheel_mc_.SetRPM();
        steering_mc_.SetRPM();
        break;
      default:  // case MC_DRIVER_STATE_DEFAULT
        if (estop_state_)
        {
          ROS_WARN("MCDriver: Switching from DEFAULT to ESTOP");
          state_ = MC_DRIVER_STATE_ESTOP;
          reset_ticks_ = true;
          break;
        }
        if (manual_override_)
        {
          ROS_WARN("MCDriver: Switching from DEFAULT to MANOVRD");
          state_ = MC_DRIVER_STATE_MANUAL_OVERRIDE;
          break;
        }
        if (steering_mc_.calibration_)
        {
          ROS_WARN("MCDriver: Switching from DEFAULT to CALIBRATE");
          state_ = MC_DRIVER_STATE_CALIBRATE;
          PublishCalibrationStatus(true);
          break;
        }
        if (CheckVelocityCommandStatus())
        {
          ROS_DEBUG_STREAM_THROTTLE(
                0.5, "Motor command expired, setting velocities to 0");
          wheel_mc_.RPM_cmd_   = 0;
        }
        wheel_mc_.SetRPM();
        steering_mc_.SetRPM();
        break;
      }

      ROS_DEBUG_THROTTLE(1, "MCState: %d", state_);

      CheckMeasurements();

      // ROS_DEBUG_STREAM("Loop in " << (can_socket_.GetCurrentTimeStamp()-start) << " ms");

      rate.sleep();  // Throttle down to 40Hz
    }

    // Attempt to reconnect if exited because of error
    if (status_count_ != 0)
    {
      // attempt to reset the CAN socket
      ROS_WARN_STREAM("Attempting to reset socket. status_count: " << status_count_);
      int status = can_socket_.ResetSocket();
      ROS_WARN_STREAM("Reset CAN socket with status code of " << status);

      // Re-enter the loop
      exit_driver_loop_ = false;

      // reset failure counter
      status_count_ = 0;
    }
  }
  // Exit thread
  can_socket_.CloseSocket();
}

/**
 * @name    GetMotorVoltage
 * @brief   Returns front motor controller voltage.
 */
double BicycleMCDriver::GetMotorVoltage()
{
  return steering_mc_.voltage_;
}


/**
 * @name  InitDriver
 * @brief This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int BicycleMCDriver::InitDriver(ros::NodeHandle *node_handle)
{
  int connect_status = AvidbotsMCCANOpenDriver::InitDriver(node_handle);

  // Start driver thread
  if (connect_status == SocketCANOpen::kCANOpenSuccess)
  {
    driver_thread_ = new boost::thread(
          boost::bind(&BicycleMCDriver::DriverThread, this));
  }

  if (connect_status == SocketCANOpen::kCANOpenSuccess)
  {
    diagnostics_timer_ = node_handle_->createTimer
      (ros::Duration(avidbots_diagnostics_constants::UPDATE_FREQ), &BicycleMCDriver::UpdateDiagnostics, this);
    diagnostics_timer_.start();
  }

  return connect_status;
}

/**
 * @name  StartPublishStatusTimer
 * @brief Starts the status publisher timer.  Main loop.
 */
void BicycleMCDriver::StartPublishStatusTimer()
{
  publish_status_timer_ = node_handle_->createTimer
      (ros::Duration(1/status_pub_rate_),
       &BicycleMCDriver::PublishStatusCallBack, this);

  ros::Rate rate(status_pub_rate_);
  while (!exit_driver_)
  {
    rate.sleep();
    ros::spinOnce();
  }
}

/**
 * @name    CheckMeasurements
 * @brief   Checks measurements in supporting motor drivers.
 */
void BicycleMCDriver::CheckMeasurements()
{
  /* The non critical measurements are measured in a round robin fashion:
   * Each non critical measurement ha s a specific ID relative to its motor position.
   * The sequence variable contains a number from 0 to sequence_max
   * The sequence_so_far keeps track of the specific measurement ID
   * When the sequence number-sequence_so_far equals the non critical
   * measurement ID, the measurement is taken
   */
  sequence_ = (sequence_ + 1) % sequence_max_;
  int seq = sequence_;

  status_count_ += wheel_mc_.RefreshReadings(seq);
  seq           -= wheel_mc_.non_critical_to_update_;
  status_count_ += steering_mc_.RefreshReadings(seq);
  seq           -= steering_mc_.non_critical_to_update_;
  if (mc_sq_pos_control_)
  {
    status_count_ += sq_pos_mc_.RefreshReadings(seq);
    seq           -= sq_pos_mc_.non_critical_to_update_;
  }
  if (mc_ch_pos_control_)
  {
    /* Motor controller requires CH pos command to be updated no faster than
     * every 70ms or no slower than every 80ms. Hence, CH pos update has
     * a separate sequence count. (SYS-1922) */
    ch_pos_update_sequence_ = (ch_pos_update_sequence_ + 1) %
                                  ch_pos_mc_.non_critical_to_update_;
    status_count_ += ch_pos_mc_.RefreshReadings(ch_pos_update_sequence_);
  }

  ROS_DEBUG_STREAM("Amps Wheel Drive Motor: " << wheel_mc_.amps_ <<
                   " Amp Steering Motor L: " << steering_mc_.amps_);
  ROS_DEBUG_STREAM("MC Reading Sequence " << sequence_ <<
                   ", max: " << sequence_max_);

  if (status_count_ != 0)
  {
    // there were connection errors in this run
    // disconnect from MC and connect again.
    ROS_WARN_STREAM("DriverLoop: CheckMeasurements: There were errors: " <<
                     status_count_);
    exit_driver_loop_ = true;

    // publish mc diagnostics states
    avidbots_msgs::diagnostics_states mc_diagnostics_states_msg;
    mc_diagnostics_states_msg.stamp = ros::Time::now();
    mc_diagnostics_states_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCDriver;
    mc_diagnostics_states_msg.states = avidbots_msgs::diagnostics_states::kStateError;
    mc_diagnostics_states_msg.description = "MCexit.";
    mc_diagnostics_states_pub_.publish(mc_diagnostics_states_msg);
  }
}

/**
 * @name    PublishCalibrationStatus
 * @brief   Publishes updated calibration status
 * @param[in] status: True if the steering wheel has started calibration.
 */
void BicycleMCDriver::PublishCalibrationStatus(const bool& status)
{
  std_msgs::Bool calib_status;
  calib_status.data = status;
  calib_status_pub_.publish(calib_status);
}

