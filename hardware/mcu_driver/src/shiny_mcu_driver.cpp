/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    shiny_mcu_driver.cpp
 * @brief	Source File containing the Avidbots MCU driver class
 * @author	Jake Park
 */

#include "avidbots_mcu_driver/shiny_mcu_driver.h"

// Local
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_msgs/mcu_status.h"

// ROS
#include "std_msgs/Bool.h"


/**
 * @name 	InitDriver
 * @brief	This function initializes the driver. Subcribes/publishes
          and retrieves the parameter settings
 * @param[in] private_nh: The private node handle
 * @param[in] node_handle: The node handle
 * @return The result of connection to the CAN socket
 */
int ShinyMCUDriver::InitDriver(const ros::NodeHandle &private_nh,
                                ros::NodeHandle *node_handle)
{
  int status = AvidbotsMCUDriver::InitDriver(private_nh, node_handle);

  return status;
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 */
void ShinyMCUDriver::GetParamSettings(const ros::NodeHandle &private_nh)
{
  AvidbotsMCUDriver::GetParamSettings(private_nh);

  GetParamUtil::GetParam("/mcu_properties/driver_loop_rate",
                         driver_loop_rate_, 10);
}

/**
 * @name    PublishStatusLoop
 * @brief Publish the status of the variables
 */
void ShinyMCUDriver::StartPublishStatusLoop()
{
  int failure_count = 0;
  avidbots_msgs::mcu_status   status_msg;
  std_msgs::Bool msg_dropped_msg;
  ros::spinOnce();
  ros::Rate rate(driver_loop_rate_);  // 10 Hz

  while (exit_driver_ == false)
  {
    while (exit_driver_loop_ == false && exit_driver_ == false)
    {
      // Get variable status from MCU board
      failure_count += can_controller_.GetStatusVariable(mcu_state_variable_,
                                      mcu_estop_status_);
      if (failure_count != 0)
      {
        // There were connection errors in this run
        // Disconnect from MCU and connect again.
        ROS_DEBUG_STREAM("MCU Driver loop: There were errors");
        exit_driver_loop_ = true;
        // Publish that there was a dropped msg
        msg_dropped_msg.data = true;
        mcu_dropped_msg_pub_.publish(msg_dropped_msg);
      }
      else
      {
        // Write output commands to CAN
        can_controller_.WritePendingMessages();
        // Publish that there was no dropped msg
        msg_dropped_msg.data = false;
        mcu_dropped_msg_pub_.publish(msg_dropped_msg);
      }

      status_msg.stamp                = ros::Time::now();
      status_msg.state_variable       = mcu_state_variable_;
      status_msg.estop_status         = mcu_estop_status_;
      mcu_status_pub_.publish(status_msg);

      // Update other nodes on the emergency,warning off state status
      PublishMCUStateUpdate(mcu_state_variable_);

      rate.sleep();
      ros::spinOnce();
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
