/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  ROS Interface
 * @brief The main header for Safety Monitor ROS interface Class
 * @author  Keshav Iyengar
 */

// ROS
#include <ros/ros.h>

// Local
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/messengers/safetyzone_messenger.h"
#include "avidbots_safety_monitor/messengers/low_voltage_messenger.h"

// MSGS
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/mcu_output_cmd.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <avidbots_msgs/sm_status.h>

// UTILS
#include <avidbots_library/get_param/get_param_util.h>
#include <avidbots_library/socket_can/avidbots_can_open_constants.h>


/******************
 * INIT FUNCTIONS *
 ******************/

/**
 * @name    ROSInterface()
 * @brief   Default constructor
 */
ROSInterface::ROSInterface(SMManager* manager): manager_(manager)
{
  node_start_time_      = ros::Time::now();
  diagnostics_start_time_ = ros::Time::now();
}

/**
 * @name        ROSInterface
 * @brief       Destructor.
 */
ROSInterface::~ROSInterface()
{
  delete safetyzone_messenger_;
  delete low_voltage_messenger_;
  delete diagnostics_messenger_;
}

/**
 * @name    Initialize()
 * @brief   Initialization function
 */
void ROSInterface::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor ROS Interface ***");

  safetyzone_messenger_ = new SafetyZoneMessenger(this);
  safetyzone_messenger_->Initialize();
  low_voltage_messenger_ = new LowVoltageMessenger(this);
  low_voltage_messenger_->Initialize();
  diagnostics_messenger_ = new DiagnosticsMessenger(this);
  diagnostics_messenger_->Initialize();


  GetParamSettings();
  InitializePublishersAndSubscribers();
  InitializeTimers();
}

/**
 * @name    GetParamsSettings()
 * @brief   Sets parameter data members according to values from ROS param server
 */
void ROSInterface::GetParamSettings()
{
  GetParamUtil::GetParam("/sm_properties/enable_safety_monitor", enable_safety_monitor_, true);
  manager_->SetEnableSafetyMonitor(enable_safety_monitor_);
}

/**
 * @name  InitializePublishersAndSubscribers
 * @brief Manages publishers and subscribers
 */
void ROSInterface::InitializePublishersAndSubscribers()
{
  /* Publishers */
  ROS_DEBUG_STREAM("*** Publishing to the vel_muxer message ***");
  vel_muxer_pub_      = node_.advertise<geometry_msgs::Twist>
                        (avidbots_topics::cmd_vel_mux_input_safety, 1, true);

  ROS_DEBUG_STREAM("*** Publishing Safety Monitor State ***");
  sm_state_pub_       = node_.advertise<std_msgs::String>
                        (avidbots_topics::sm_state, 10, true);

  ROS_DEBUG_STREAM("*** Publishing Safety Monitor Status ***");
  sm_status_pub_       = node_.advertise<avidbots_msgs::sm_status>
                        (avidbots_topics::sm_status_topic, 10, true);

  ROS_DEBUG_STREAM("*** Publishing Safety Monitor State ***");
  sm_error_pub_       = node_.advertise<std_msgs::String>
                        (avidbots_topics::sm_error_topic, 10, true);

  ROS_DEBUG_STREAM("*** Publishing Handshake percentage ***");
  succ_fail_handshake_pub_ = node_.advertise<std_msgs::Float32>
                             (avidbots_topics::sm_handshake_percentage, 10, true);

  ROS_DEBUG_STREAM("*** Publishing MCU Output command message ***");
  mcu_output_pub_     = node_.advertise<avidbots_msgs::mcu_output_cmd>
                        (avidbots_topics::mcu_output_cmd_msg, 10, true);

  ROS_DEBUG_STREAM("*** Publishing sensor safety enable message ***");
  sensor_safety_pub_     = node_.advertise<avidbots_msgs::sensor_toggle>
                        (avidbots_topics::ss_sensor_status, 1, true);

  /* Subscribers */
  ROS_DEBUG("*** Subscribing to MCU Status message ***");
  mcu_status_sub_     = node_.subscribe(avidbots_topics::mcu_status_msg, 1,
                        &ROSInterface::MCUStatusCallBack, this);

  ROS_DEBUG("*** Subscribing to MC States message ***");
  mc_states_sub_     = node_.subscribe(avidbots_topics::mc_states_topic, 1,
                        &ROSInterface::MCStatesCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the safety monitor failure reset message ***");
  sm_failure_reset_sub_ = node_.subscribe(avidbots_topics::sm_failure_reset_topic, 1,
                          &ROSInterface::SmFailureResetCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the UI state message ***");
  ui_state_sub_ = node_.subscribe(avidbots_topics::ui_state_topic, 1,
                &ROSInterface::UIStateCallback, this);
}

/**
 * @name  InitializeTimers
 * @brief Manages Timers
 */
void ROSInterface::InitializeTimers()
{
  /* Create main monitor timer */
  monitor_timer_ = node_.createTimer
      (ros::Duration(0.1), &ROSInterface::MainMonitorTimerCallBack, this);

        /* Create handshake timer */
  mcu_handshake_timer_ = node_.createTimer
      (ros::Duration(0.1), &ROSInterface::MCUHandshakeTimerCallBack, this);

  safetyzone_messenger_->InitializeTimers();
  low_voltage_messenger_->InitializeTimers();
  diagnostics_messenger_->InitializeTimers();
}

/*************
 * CALLBACKS *
 *************/

/**
 * @name    MainMonitorTimerCallBack()
 * @brief   The main timer callback function tat performs state transitions
 *          and executes functions based on state.
 */
void ROSInterface::MainMonitorTimerCallBack(const ros::TimerEvent &)
{
  manager_->Update();
  if (ros::Time::now() - node_start_time_ > ros::Duration(NODE_START_DELAY))
  {
    if ((mcu_state_ == MCU_STATE_VAR_NORMAL)
       && (mc_state_ != avidbots_msgs::mc_states::kESTOP)
       && (mc_state_ != avidbots_msgs::mc_states::kCALIBRATE))
    {
      if (ros::Time::now() - diagnostics_start_time_ > ros::Duration(DIAGNOSTICS_START_DELAY))
      {
        // enable diagnostics & sensor safety toggling
        PublishSensorSafetyToggle(true);
      }
    }
    else
    {
      diagnostics_start_time_ = ros::Time::now();
      // disable diagnostics & sensor safety toggling
      PublishSensorSafetyToggle(false);
    }
  }
}

/**
 * @name    MCUHandshakeTimerCallBack()
 * @brief   Attempts to send and receive a handshake with mcu driver.
            Will update the mcu_state_ correspondingly. */

void ROSInterface::MCUHandshakeTimerCallBack(const ros::TimerEvent &)
{
  if (ros::Time::now() - node_start_time_ > ros::Duration(NODE_START_DELAY))
  {
    if (manager_->GetEnableSafetyMonitor())
    {
      /* Attempt to read heartbeat */
      PublishMCUOutputCommand(SAFETY_MONITOR_NORMAL);
    }
  }
}

/**
 * @name  MCUStatusCallBack
 * @brief Call back for the mcu status msg.
 * @param[in] mcu_status_msg: The MCU status message
 */
void ROSInterface::MCUStatusCallBack(const avidbots_msgs::mcu_status& mcu_status_msg)
{
  mcu_state_ = mcu_status_msg.state_variable;
  if (mcu_state_ != MCU_STATE_VAR_NORMAL)
  {
      diagnostics_start_time_ = ros::Time::now();
      // disable diagnostics & sensor safety toggling
      PublishSensorSafetyToggle(false);
  }
  if (mcu_status_msg.state_variable == MCU_STATE_VAR_EMERGENCY)  // In EStop
  {
    manager_->SetEStopStatus(ERROR);
    manager_->SMStateEvent(ESTOP_EVENT);
  }
  else
  {
    manager_->SetEStopStatus(NO_ERROR);
  }
}

/**
 * @name  MCStatesCallBack
 * @brief Call back for the mc states msg.
 * @param[in] mc_states_msg: The MC states message
 */
void ROSInterface::MCStatesCallBack(const avidbots_msgs::mc_states& mc_states_msg)
{
  mc_state_ = mc_states_msg.state;
  if ((mc_state_ == avidbots_msgs::mc_states::kESTOP)
       || (mc_state_ == avidbots_msgs::mc_states::kCALIBRATE))
  {
    diagnostics_start_time_ = ros::Time::now();
    // disable diagnostics & sensor safety toggling
    PublishSensorSafetyToggle(false);
  }
}

/**
 * @name    SmFailureResetCallBack()
 * @brief   Call back for safety monitor failure reset
 * @param[in] sm_failure_reset_msg: true if the safety monitor failure is reset
 */
void ROSInterface::SmFailureResetCallBack(
          const std_msgs::Bool& sm_failure_reset_msg)
{
  sm_failure_reset_ = sm_failure_reset_msg.data;
  manager_->SetSmFailureReset(sm_failure_reset_);
  manager_->SMStateEvent(FAILURE_RESET_EVENT);
}

/**
 * @name    UIStateCallback()
 * @brief   Call back for safety monitor failure reset
 * @param[in] sm_failure_reset_msg: true if the safety monitor failure is reset
 */
void ROSInterface::UIStateCallback(
          const avidbots_msgs::ui_state& ui_state_msg)
{
  manager_->SetUIState(ui_state_msg.state);
}

/*****************************
 * STATE EXECUTION FUNCTIONS *
 *****************************/
/**
 * @name    PublishZeroVelocity()
 * @brief   Publishes zero velocity to mobile/base and vel_cmd_muxer
 */
void ROSInterface::PublishZeroVelocity()
{
  geometry_msgs::Twist zero_vel;
  zero_vel.angular.z = 0.0;
  zero_vel.linear.x = 0.0;
  zero_vel.linear.y = 0.0;

  vel_muxer_pub_.publish(zero_vel);
  // ROS_WARN("Velocities have been set to 0");
}

/************************
 * Calls from State Base*
 ************************/

/**
 * @name    PublishErrorMsg()
 * @brief   Publishes an error message
 * @param[in] error msg: msg to publish
 */
void ROSInterface::PublishErrorMsg(std_msgs::String error_msg)
{
  sm_error_pub_.publish(error_msg);
}

/**
 * @name    PublishSMStatus()
 * @brief   Publishes an sm status message
 * @param[in] sm status: status to update GUI with
 */
void ROSInterface::PublishSMStatus(avidbots_msgs::sm_status sm_status)
{
  sm_status_pub_.publish(sm_status);
}

/**
 * @name    PublishStateMsg()
 * @brief   Publishes a state message
 * @param[in] state_msg_: msg to publish
 */
void ROSInterface::PublishStateMsg(std_msgs::String state_msg_)
{
  sm_state_pub_.publish(state_msg_);
}

/**
 * @name    PublishDangerZoneObstacle
 * @brief   Sets the safety monitor reset status
 * @param[in] status: status to publish
 */
void ROSInterface::PublishDangerZoneObstacle(bool status)
{
  safetyzone_messenger_->PublishDangerZoneObstacle(status);
}

/**
 * @name    PublishMCUOutputCommand()
 * @brief   Sets MCU to Safety zone EState
 */
void ROSInterface::PublishMCUOutputCommand()
{
  safetyzone_messenger_->PublishMCUOutputCommand();
}

/**
 * @name      PublishMCUOutputCommand()
 * @brief     Publish output command to MCU Driver
 * @param[in] value: value of MCU output command to publish
 */
void ROSInterface::PublishMCUOutputCommand(const int &value)
{
  avidbots_msgs::mcu_output_cmd output_cmd;
  output_cmd.output_num = avidbots_msgs::mcu_output_cmd::kSafetyMonitor;
  output_cmd.output_value  = value;

  mcu_output_pub_.publish(output_cmd);
}

/**
 * @name      PublishSensorSafetyToggle()
 * @brief     Publish Sensor Safety Toggle
 * @param[in] value: enable or disable
 */
void ROSInterface::PublishSensorSafetyToggle(const bool &value)
{
  avidbots_msgs::sensor_toggle msg;
  msg.stamp = ros::Time::now();
  msg.sensor_id = avidbots_msgs::sensor_toggle::kGlobal;
  msg.status  = value;
  sensor_safety_pub_.publish(msg);
}
