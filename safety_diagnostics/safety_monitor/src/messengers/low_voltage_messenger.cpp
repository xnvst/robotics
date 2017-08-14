/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  low_voltage_messenger
 * @brief low_voltage_messenger cpp file contains the low_voltage_messenger
 * @author Keshav Iyengar
 */


// LOCAL
#include <avidbots_msgs/topics.h>
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/messengers/low_voltage_messenger.h"

#include <avidbots_library/get_param/get_param_util.h>

/******************
 * LOCAL CONSTANTS *
 ******************/

const uint8_t kNumLowVoltageMeasurements = 60;

/**
 * @name        LowVoltageMessenger
 * @brief       Constructor
 * @param[in]   ros: a pointer to the ROSInterface it communicates with
 */
LowVoltageMessenger::LowVoltageMessenger(ROSInterface* ros)
{
  ros_ = ros;
}

/**
 * @name        initialize
 * @brief       initialize publishers and subscribers used
 */
void LowVoltageMessenger::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor Low Voltage Messenger ***");
  voltage_low_counter_ = 0;
  GetParamSettings();
  InitializePublishersAndSubscribers();
}

/**
 * @name        GetParamSettings
 * @brief       Sets parameter data members according to values from ROS param server
 */
void LowVoltageMessenger::GetParamSettings()
{
  ROS_INFO_STREAM("*** Low Voltage Messenger getting Parameters ***");
  GetParamUtil::GetParam("/sm_properties/enable_low_voltage_monitor", enable_low_voltage_monitor_, false);
  ros_->manager_->SetEnableLowVoltage(enable_low_voltage_monitor_);
  GetParamUtil::GetParam("/preferences/battery/voltage/min", kMinVoltage_, 32.0);
  GetParamUtil::GetParam("/preferences/battery/voltage/critical", kCriticalVoltage_, 32.76);
}

/**
 * @name  InitializePublishersAndSubscribers
 * @brief Manages publishers and subscribers
 */
void LowVoltageMessenger::InitializePublishersAndSubscribers()
{
  ROS_INFO_STREAM("*** Low Voltage Messenger Initializing subs and pubs ***");
  mc_voltage_status_sub_ = node_.subscribe(avidbots_topics::mc_voltage_status_topic, 1,
                                           &LowVoltageMessenger::MCVoltageStatusCallBack, this);
}

/**
 * @name  InitTimers
 * @brief Manages Timers
 */
void LowVoltageMessenger::InitializeTimers()
{
  ROS_INFO_STREAM("*** Low Voltage Messenger Initializing timers ***");
    /* Create low voltage timer */
  voltage_timer_ = node_.createTimer
      (ros::Duration(1.0), &LowVoltageMessenger::LowVoltageTimerCallBack, this);
}



/**
 * @name  MCVoltageStatusCallBack
 * @brief Call back for the mc voltage status msg.
 * @param[in] voltage_status_msg: The MC voltage status message
 */
void LowVoltageMessenger::MCVoltageStatusCallBack(const avidbots_msgs::mc_voltage_statusPtr&
                                                        voltage_status_msg)
{
  // Update internal voltage variable
  current_voltage_ = voltage_status_msg->batt_voltage;
  // If state of charge is 0, invalidate voltage measurement
  if (voltage_status_msg->state_of_charge >= 0.01)
  {
    voltage_is_valid_ = true;
  }
  else
  {
    voltage_is_valid_ = false;
  }
}

/**
 * @name    LowVoltageTimerCallBack()
 * @brief   The low voltage monitor call back
 */
void LowVoltageMessenger::LowVoltageTimerCallBack(const ros::TimerEvent &)
{
  /*
   *  This function checks that the votlage is low
   * It checks that the state of charge is more than 0 which means
   * its a valid measurement
   * It also checks that the low voltage has been present for at least
   * 10 measurements.
   * */
  if (ros_->manager_->GetEnableSafetyMonitor() && ros_->manager_->GetEnableLowVoltage())
  {
    if ((current_voltage_ < kCriticalVoltage_) && voltage_is_valid_)
    {
      voltage_low_counter_ = voltage_low_counter_ + 1;
      if (voltage_low_counter_ > kNumLowVoltageMeasurements)
      {
        ROS_WARN_STREAM("Voltage is too low");
        ros_->manager_->SetLowVoltageStatus(ERROR);
        ros_->manager_->SMStateEvent(LOW_VOLTAGE_EVENT);
      }
    }
    else
    {
      ros_->manager_->SetLowVoltageStatus(NO_ERROR);
      voltage_low_counter_ = 0;
    }
  }
  else
  {
    ros_->manager_->SetLowVoltageStatus(NO_ERROR);
    voltage_low_counter_ = 0;
  }
}
