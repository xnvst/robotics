/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  avidbots_mc_driver.cpp
 * @brief Source File containing the Avidbots MC driver class
 * @author  Pablo Molina, Jake Park
 */

// CPP
#include <string>

// LOCAL
#include "avidbots_mc_driver/avidbots_mc_driver.h"
#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_mc_driver/mc_voltage_helper.h"
#include <avidbots_msgs/mc_status.h>
#include <avidbots_msgs/mc_voltage_status.h>
#include <avidbots_msgs/mcu_status.h>
#include <avidbots_msgs/topics.h>

/**
 * @name  AvidbotsMCCANOpenDriver
 * @brief Default constructor
 */
AvidbotsMCCANOpenDriver::AvidbotsMCCANOpenDriver()
{
  voltage_pub_rate_           = 1;
  exit_driver_                = false;
  exit_driver_loop_           = false;
  manual_override_            = false;
  estop_state_                = false;
  state_                      = MC_DRIVER_STATE_DEFAULT;

  voltage_data_.set_capacity(kVoltageBufferSize);
}

/**
 * @name  ~AvidbotsBase
 * @brief Destructor
 */
AvidbotsMCCANOpenDriver::~AvidbotsMCCANOpenDriver()
{
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void AvidbotsMCCANOpenDriver::ManagePubAndSub()
{
  // publish
  ROS_INFO_STREAM("*** Publishing motor controller status message ***");
  mc_status_pub_     = node_handle_->advertise
                       <avidbots_msgs::mc_status>
                       (avidbots_topics::mc_status_topic, 1, true);

  ROS_INFO_STREAM("*** Publishing motor controller voltage status message ***");
  mc_voltage_pub_    = node_handle_->advertise
                       <avidbots_msgs::mc_voltage_status>
                       (avidbots_topics::mc_voltage_status_topic, 10, true);

  ROS_INFO_STREAM("*** Publishing the MCDiagnostics message ***");
  mc_diagnostics_states_pub_     = node_handle_->advertise
                    <avidbots_msgs::diagnostics_states>
                    (avidbots_topics::diagnostics_states_topic, 1, true);

  // subscribe
  ROS_INFO_STREAM("*** Subscribing for the manual override message ***");
  manual_override_sub_ = node_handle_->
                         subscribe(avidbots_topics::mc_manual_override_msg, 5,
                         &AvidbotsMCCANOpenDriver::ManualOverrideCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the mcu estop command message ***");
  mcu_estop_cmd_sub_  = node_handle_->
                      subscribe(avidbots_topics::mcu_estop_status_msg, 10,
                      &AvidbotsMCCANOpenDriver::MCUEstopCallBack, this);
}

/**
 * @name  ConnectToCanSocket
 * @brief Attempts to connect to the CAN Socket every second
          This is a locking call and will not return unless required
 * @param[in] port: The port name to connect to
 * @return The result of connection to the can socket device
 */

int AvidbotsMCCANOpenDriver::ConnectToCanSocket(const std::string &port)
{
  int status = SocketCANOpen::kCANOpenSuccess;

  // Repeat trying to connect to the CAN Socket
  while (exit_driver_ == false)
  {
    ROS_INFO_STREAM("*** Connecting to CAN Socket on port: "
                    << port << " ***");
    status = can_socket_.InitSocket(port, can_timeout_);
    if (status != SocketCANOpen::kCANOpenSuccess)
    {
      ROS_ERROR_STREAM("*** Error connecting to CAN Socket. Status: "
                       << status << ". Retrying again every second ... ***");
      // Something went wrong
      connection_status_ = false;
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
 * @name  InitTimers
 * @brief Initialize timers
 */
void AvidbotsMCCANOpenDriver::InitTimers()
{
  publish_voltage_status_timer_ = node_handle_->createTimer
      (ros::Duration(voltage_pub_rate_),
       &AvidbotsMCCANOpenDriver::PublishVoltageCallBack, this);
}

/**
 * @name  InitDriver
 * @brief This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int AvidbotsMCCANOpenDriver::InitDriver(ros::NodeHandle *node_handle)
{
  node_handle_        = node_handle;
  motor_cmd_time_     = can_socket_.GetCurrentTimeStamp();
  int connect_status  = SocketCANOpen::kCANOpenSuccess;

  // Get parameters
  GetParamSettings();

  // Manage publishers and subscribers
  ManagePubAndSub();

  // Initialize motor objects
  InitMotorObjects();

  // Initialize timers
  InitTimers();

  // Attemping to connect to the CAN socket
  connect_status = ConnectToCanSocket(can_port_name_);

  return connect_status;
}

/**
 * @name  ExitDriverThread
 * @brief Stops all processing threads
 */
void AvidbotsMCCANOpenDriver::ExitDriverThread()
{
  exit_driver_ = true;
}

/**
 * @name ManualOverrideCallback
 * @brief Read in boolean topic messages to set manual override status
 * @param manual_override_cmd The boolean status
 */
void AvidbotsMCCANOpenDriver::ManualOverrideCallBack(
    const std_msgs::BoolPtr& manual_override_msg)
{
  // static cast because Bool type messages contain uint8s
  manual_override_ = static_cast<bool>(manual_override_msg->data);
  ROS_DEBUG_STREAM("MC Driver: Received manual override: " << manual_override_);
}

/**
 * @name  MCUEstopCallBack
 * @brief Function that updates local variable mcu_estop_state accoring to
 *        the MCU estop state / Sets the mc cmd to zero.
 * @param[msg]  : The incoming message
 */
void AvidbotsMCCANOpenDriver::MCUEstopCallBack(const std_msgs::Bool &msg)
{
  estop_state_ = static_cast<bool>(msg.data);
}

/**
 * @name  PublishVoltageCallBack
 * @brief Publish the status of voltage and state of charge
 */
void AvidbotsMCCANOpenDriver::PublishVoltageCallBack(
    const ros::TimerEvent& timer_event)
{
  avidbots_msgs::mc_voltage_status voltage_status_msg;
  motor_voltage_ = GetMotorVoltage();

  double battery_voltage =
      mc_voltage_helper::GetAverageVoltage(voltage_data_, motor_voltage_);

  /* The relation between SOC and voltage is approximately linear */
  double soc = ((battery_voltage - kMinVoltage) / (kMaxVoltage - kMinVoltage)) * 100;

  // Saturate percent
  soc = soc < 0 ? 0 : (soc > 100 ? 100 : soc);

  /* The front mc voltage is 10 times the battery voltage */
  voltage_status_msg.batt_voltage = battery_voltage / 10.0;
  voltage_status_msg.state_of_charge = soc;

  if (motor_voltage_ > 0)
  {
    /* Push back latest voltage info and start calculations */
    voltage_data_.push_back(static_cast<int>(motor_voltage_));
  }

  mc_voltage_pub_.publish(voltage_status_msg);
}

/**
 * @name    checkVelocityCommandStatus
 * @brief   Check if the motor controller speed
            commands are being updated
 * @return  True if the motor controller speed
            commands are not being updated - false otherwise
 */
bool AvidbotsMCCANOpenDriver::CheckVelocityCommandStatus()
{
  return (can_socket_.GetCurrentTimeStamp() > motor_cmd_time_
           + controller_cmd_timeout_*1000);
}

