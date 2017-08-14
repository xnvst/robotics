/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    wheely_mcu_driver.cpp
 * @brief	Source File containing the Avidbots MCU driver class
 * @author	Jake Park
 */

#include "avidbots_mcu_driver/wheely_mcu_driver.h"

// Local
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_msgs/mcu_status.h"
#include "avidbots_msgs/water_sensor_status.h"
#include <avidbots_msgs/topics.h>


// ROS
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

/**
 * @name 	InitDriver
 * @brief	This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int WheelyMCUDriver::InitDriver(const ros::NodeHandle &private_nh,
                                ros::NodeHandle *node_handle)
{
  int status = AvidbotsMCUDriver::InitDriver(private_nh, node_handle);

  return status;
}

void WheelyMCUDriver::ManagePubAndSub()
{
  AvidbotsMCUDriver::ManagePubAndSub();

  water_sensor_status_pub_ = node_handle_->advertise<avidbots_msgs::water_sensor_status>
                                           (avidbots_topics::mcu_water_sensor_status_msg, 1, true);
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 */
void WheelyMCUDriver::GetParamSettings(const ros::NodeHandle &private_nh)
{
  AvidbotsMCUDriver::GetParamSettings(private_nh);

  GetParamUtil::GetParam("/mcu_properties/driver_loop_rate",
                         driver_loop_rate_, 10);
}

/**
 * @name    ManageMCUVersion
 * @brief   Read and Publish MCU Version
 * @return  0 for get MCU verion ok, -1 for not ok  
 */
int WheelyMCUDriver::ManageMCUVersion(void)
{
  if (mcu_version_cnt_ < MAX_MCU_VERSION_CNT_ && mcu_version_ok_ != can_controller_.kSuccess)
  {
    mcu_version_cnt_++;
    mcu_version_ok_ = can_controller_.GetMCUFirmwareVersion(ver_major_, ver_minor_, ver_patch_);
  }
  else
  {
    avidbots_msgs::mcu_firmware_version fw_version_msg;
    fw_version_msg.major_val = ver_major_;
    fw_version_msg.minor_val = ver_minor_;
    fw_version_msg.patch_val = ver_patch_;
    mcu_firmware_version_pub_.publish(fw_version_msg);
  }
  return mcu_version_ok_;
}


/**
 * @name    ManageManualMotion
 * @brief   Read and Publish Manual Motion
 * @return  The status of the command sent on CAN bus to get Manual Motion
 *          Value defined in socket_can_open_constants.h
 */
int WheelyMCUDriver::ManageManualMotion(void)
{
  int ret = 0;
  avidbots_msgs::mcu_manualmotion manualmotion_msg;
  if (AvidbotsMCUDriver::gearmode_state_ == avidbots_msgs::gearmode::kParkingMode)
  {
    return ret;
  }
  // get manual motion variable from MCU board and publish
  ret = can_controller_.GetManualMotionVariable(steering_val_, driving_val_);
  if (ret != 0)
  {
    return ret;
  }
  manualmotion_msg.stamp                = ros::Time::now();
  manualmotion_msg.gearmode_val         = AvidbotsMCUDriver::gearmode_state_;
  if (AvidbotsMCUDriver::gearmode_state_ == avidbots_msgs::gearmode::kNeutralMode)
  {
    manualmotion_msg.steering_val         = 0;
    manualmotion_msg.driving_val          = 0;
  }
  else
  {
    manualmotion_msg.steering_val         = steering_val_;
    manualmotion_msg.driving_val          = driving_val_;
  }
  mcu_manualmotion_pub_.publish(manualmotion_msg);
  return ret;
}

/**
 * @name    ManageStatusVariable
 * @brief   Read and Publish Status Variable
 * @return  The status of the command sent on CAN bus to get Status Variable
 *          Value defined in socket_can_open_constants.h
 */
int WheelyMCUDriver::ManageStatusVariable(void)
{
  int ret = 0;
  avidbots_msgs::mcu_status status_msg;

  // Get variable status from MCU board
  ret = can_controller_.GetStatusVariable(clean_water_empty_state_,
                                                         dirty_water_full_state_,
                                                         mcu_estop_status_,
                                                         mcu_state_variable_);

  status_msg.stamp                = ros::Time::now();
  status_msg.state_variable       = mcu_state_variable_;
  status_msg.estop_status         = mcu_estop_status_;
  mcu_status_pub_.publish(status_msg);

  water_sensor_status_msg_.clean_water_empty_state = clean_water_empty_state_;
  water_sensor_status_msg_.dirty_water_full_state  = dirty_water_full_state_;

  water_sensor_status_pub_.publish(water_sensor_status_msg_);

  // Update other nodes on the emergency,warning off state status
  PublishMCUStateUpdate(mcu_state_variable_);

  return ret;
}

/**
 * @name    ManagePendingMessages
 * @brief   Write Pending Messges for output cmd
 * @return  The status of the command sent on CAN bus to set Output cmd
 *          Value defined in socket_can_open_constants.h
 */
int WheelyMCUDriver::ManagePendingMessages(void)
{
  return can_controller_.WritePendingMessages();
}

/**
 * @name    PublishStatusLoop
 * @brief Publish the status of the variables
 */
void WheelyMCUDriver::StartPublishStatusLoop()
{
  int failure_count = 0;
  std_msgs::Bool msg_dropped_msg;
  ros::spinOnce();
  ros::Rate rate(driver_loop_rate_);  // 10 Hz

  while (exit_driver_ == false)
  {
    while (exit_driver_loop_ == false && exit_driver_ == false)
    {
      rate.sleep();
      ros::spinOnce();

      ManageMCUVersion();

      failure_count += ManageManualMotion();

      failure_count += ManageStatusVariable();

      failure_count += ManagePendingMessages();

      if (failure_count != 0)
      {
        ROS_DEBUG_STREAM("MCU Driver loop: There were errors");
        exit_driver_loop_ = true;
        msg_dropped_msg.data = true;
        mcu_dropped_msg_pub_.publish(msg_dropped_msg);

        // publish mcu diagnostics states
        avidbots_msgs::diagnostics_states mcu_state_msg;
        mcu_state_msg.stamp = ros::Time::now();
        mcu_state_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCUDriver;
        mcu_state_msg.states = avidbots_msgs::diagnostics_states::kStateError;
        mcu_state_msg.description = "MCUexit.";
        mcu_diagnostics_states_pub_.publish(mcu_state_msg);
      }
      else
      {
        msg_dropped_msg.data = false;
        mcu_dropped_msg_pub_.publish(msg_dropped_msg);
      }
    }

    // Attempt to reconnect if exited because of error
    if (failure_count != 0)
    {
      ROS_ERROR("*** MCU failure...exited loop (failure count %i) ***",
                failure_count);
      // Re-enter the loop
      exit_driver_loop_ = false;
      failure_count = 0;
    }
  }
}
