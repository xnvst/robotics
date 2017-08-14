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
 * @brief	Source File containing the Avidbots MCU driver class
 * @author	Pablo Molina
 */
#include <ros/ros.h>
#include <string>
#include <avidbots_msgs/topics.h>
#include "avidbots_mcu_driver/mcu_driver.h"
#include "avidbots_library/socket_can/avidbots_can_open_constants.h"
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"
#include "avidbots_msgs/mcu_status.h"
#include "avidbots_msgs/mcu_output_cmd.h"
#include "std_msgs/Bool.h"

/**
 * @name 	AvidbotsMCUDriver
 * @brief	Default constructor
 */
AvidbotsMCUDriver::AvidbotsMCUDriver()
{
  exit_driver_            = false;
  exit_driver_loop_       = false;
  can_timeout_            = 100;
  mcu_estop_state_        = false;

  connection_status_ = false;
}

/**
 * @name  ~AvidbotsMCUDriver
 * @brief Destructor
 */
AvidbotsMCUDriver::~AvidbotsMCUDriver() {}


/**
 * @name 	InitDriver
 * @brief	This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int AvidbotsMCUDriver::InitDriver(const ros::NodeHandle &private_nh,
                                        ros::NodeHandle *node_handle)
{
  node_handle_        = node_handle;
  int connect_status  = SocketCANOpen::kCANOpenSuccess;

  // Initialize publishers and subscribers
  ManagePubAndSub();

  // Get parameters
  GetParamSettings(private_nh);

  // Attemping to connect to the CAN socket
  connect_status = ConnectToCanSocket(can_port_name_);

  if (connect_status == SocketCANOpen::kCANOpenSuccess)
  {
    diagnostics_timer_ = node_handle_->createTimer
      (ros::Duration(avidbots_diagnostics_constants::UPDATE_FREQ), &AvidbotsMCUDriver::UpdateDiagnostics, this);

    diagnostics_timer_.start();
  }

  return connect_status;
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 */
void AvidbotsMCUDriver::GetParamSettings
            (const ros::NodeHandle& private_nh)
{
  // Retriving MCU settings
  if (!private_nh.getParam("/mcu_properties/can_port_name",
                            can_port_name_))
    ROS_ERROR("Failed to get '/mcu_properties/can_port_name'");

  if (!private_nh.getParam("/mcu_properties/mcu_can_id",
                            mcu_can_id_))
    ROS_ERROR("Failed to get '/mcu_properties/mcu_can_id'");

  if (!private_nh.getParam("/mcu_properties/can_timeout",
                            can_timeout_))
    ROS_ERROR("Failed to get '/mcu_properties/can_timeout'");
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void AvidbotsMCUDriver::ManagePubAndSub()
{
  // subscribe
  ROS_INFO_STREAM("*** Subscribing for the MCU output command message ***");
  mcu_output_cmd_sub_    = node_handle_->
                      subscribe(avidbots_topics::mcu_output_cmd_msg, 10,
                      &AvidbotsMCUDriver::OutputCmdCallBack, this);

  ROS_INFO_STREAM("*** Subscribing for the UI gearmode message from MCU ***");
  ui_gearmode_sub_    = node_handle_->
                      subscribe(avidbots_topics::ui_gearmode_topic, 10,
                      &AvidbotsMCUDriver::UIGearmodeCallBack, this);

  // publish
  ROS_INFO_STREAM("*** Publishing the MCU status message ***");
  mcu_status_pub_        = node_handle_->advertise
                    <avidbots_msgs::mcu_status>
                    (avidbots_topics::mcu_status_msg, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU ESTOP status message ***");
  mcu_estop_status_pub_  = node_handle_->advertise
                    <std_msgs::Bool>
                    (avidbots_topics::mcu_estop_status_msg, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU Warning OFF status message ***");
  mcu_warning_off_pub_   = node_handle_->advertise
                    <std_msgs::Bool>
                    (avidbots_topics::mcu_warning_off_msg, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU Disconnection status message ***");
  mcu_disconnection_pub_ = node_handle_->advertise
                    <std_msgs::Bool>
                    (avidbots_topics::mcu_disconnection_topic, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU message drop status message ***");
  mcu_dropped_msg_pub_   = node_handle_->advertise
                    <std_msgs::Bool>
                    (avidbots_topics::mcu_dropped_msg_topic, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU Manual Motion message ***");
  mcu_manualmotion_pub_     = node_handle_->advertise
                    <avidbots_msgs::mcu_manualmotion>
                    (avidbots_topics::mcu_manualmotion_topic, 1, true);

  ROS_INFO_STREAM("*** Publishing the MCU Firmware Version message ***");
  mcu_firmware_version_pub_     = node_handle_->advertise
                    <avidbots_msgs::mcu_firmware_version>
                    (avidbots_topics::mcu_firmware_version_topic, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCU Diagnostics State message ***");
  mcu_diagnostics_states_pub_     = node_handle_->advertise
                    <avidbots_msgs::diagnostics_states>
                    (avidbots_topics::diagnostics_states_topic, 1, true);
}

/**
 * @name    PublishEstopState
 * @brief Updates all nodes with the state of the estop system
 */
void AvidbotsMCUDriver::PublishMCUStateUpdate(const uint8_t &mcu_state_var)
{
  bool mcu_estop = (mcu_state_var == MCU_STATE_VAR_EMERGENCY);
  bool mcu_warn_off = (mcu_state_var == MCU_STATE_VAR_WARNING_OFF);
  // TODO(Tony): Publish MCU disconnection message after MCU is fixed

  // Check if the MCU Estop state has changed
  if (mcu_estop != mcu_estop_state_)
  {
    std_msgs::Bool estop_status_msg;
    mcu_estop_state_      = mcu_estop;
    estop_status_msg.data = mcu_estop;
    // Send estop status change
    mcu_estop_status_pub_.publish(estop_status_msg);
  }
  if (mcu_warn_off != mcu_warning_off_state_)
  {
    std_msgs::Bool warn_off_status_msg;
    mcu_warning_off_state_   = mcu_warn_off;
    warn_off_status_msg.data = mcu_warn_off;
    // Send estop status change
    mcu_warning_off_pub_.publish(warn_off_status_msg);
  }
}

/**
 * @name 	ConnectToCanSocket
 * @brief	Attempts to connect to the CAN Socket every second
          This is a locking call and will not return unless required
 * @param[in] port: The port name to connect to
 * @return The result of connection to the can socket device
 */
int AvidbotsMCUDriver::ConnectToCanSocket(const std::string &port)
{
  int status = SocketCANOpen::kCANOpenSuccess;

  // Repeat trying to connect to the CAN Socket
  while (exit_driver_ == false)
  {
    ROS_INFO_STREAM("*** Connecting to CAN Socket on port: "
                    << port << " ***");
    status = can_controller_.Init(port, can_timeout_, mcu_can_id_);
    if (status != SocketCANOpen::kCANOpenSuccess)
    {
      ROS_ERROR_STREAM("*** Error connecting to CAN Socket. Status: "
                       << status << ". Retrying again every second ... ***");
      can_controller_.CloseSocket();
    }
    else
    {
      connection_status_ = true;
      break;  // get out of the while loop
    }
    sleep(1);  // sleeping for 1 second before retrying
  }

  if (exit_driver_ == true)
    return status;

  return SocketCANOpen::kCANOpenSuccess;
}

/**
 * @name    OuputCmdCallBack
 * @brief   Output command call Back
 * @param[in] vel_cmd: The incoming twist command
 */
void AvidbotsMCUDriver::OutputCmdCallBack
                   (const avidbots_msgs::mcu_output_cmdPtr &mcu_output_cmd)
{
  can_controller_.ProcessOutputCommand(mcu_output_cmd->output_num,
                                       mcu_output_cmd->output_value);
}

/**
 * @name    UIGearmodeCallBack
 * @brief   UI Gearmode call Back
 */
void AvidbotsMCUDriver::UIGearmodeCallBack(const avidbots_msgs::gearmodePtr &gearmode)
{
  gearmode_state_ = gearmode->value;
}

/**
 * @name    UpdateDiagnostics
 * @brief   Diagnostics function for mcu_driver.
 */
void AvidbotsMCUDriver::UpdateDiagnostics(const ros::TimerEvent &)
{
  ROS_DEBUG_STREAM("AvidbotsMCUDriver::UpdateDiagnostics\n");

  // publish mcu diagnostics states
  avidbots_msgs::diagnostics_states mcu_state_msg;
  mcu_state_msg.stamp = ros::Time::now();
  mcu_state_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCUDriver;
  mcu_state_msg.description = "";
  if (!connection_status_ || exit_driver_loop_ ||
     (mcu_state_variable_ != MCU_STATE_VAR_LOADING && mcu_state_variable_ != MCU_STATE_VAR_NORMAL))
  {
    mcu_state_msg.states = avidbots_msgs::diagnostics_states::kStateError;
    if (!connection_status_)
    {
      mcu_state_msg.description += "MCUconn.";
    }
    if (exit_driver_loop_)
    {
      mcu_state_msg.description += "MCUexit.";
    }
    if (mcu_state_variable_ != MCU_STATE_VAR_LOADING && mcu_state_variable_ != MCU_STATE_VAR_NORMAL)
    {
      mcu_state_msg.description += "MCUstate.";
    }
  }
  else
  {
    mcu_state_msg.states = avidbots_msgs::diagnostics_states::kStateOk;
  }
  mcu_diagnostics_states_pub_.publish(mcu_state_msg);
}
