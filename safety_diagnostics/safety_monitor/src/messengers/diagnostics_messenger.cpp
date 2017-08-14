/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  diagnostics_messenger
 * @brief diagnostics_messenger cpp file contains the diagnostics_messenger
 * @author Keshav Iyengar
 */


// LOCAL
#include <avidbots_msgs/topics.h>
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/messengers/diagnostics_messenger.h"
// MSG
#include <avidbots_msgs/diagnostics_states.h>

#include <avidbots_library/get_param/get_param_util.h>

/**
 * @name        DiagnosticsMessenger
 * @brief       Constructor
 * @param[in]   ros: a pointer to the ROSInterface it communicates with
 */
DiagnosticsMessenger::DiagnosticsMessenger(ROSInterface* ros)
{
  ros_ = ros;
}

/**
 * @name        initialize
 * @brief       initialize publishers and subscribers used
 *                commonly by Wheely and Shiny to communicate with ROS
 */
void DiagnosticsMessenger::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor Diagnostics Messenger ***");

  GetParamSettings();
  InitializePublishersAndSubscribers();

  // True indicates that there is no error in the status
  // False indicates that there is an error in the status
  diagnostics_status_         = NO_ERROR;
  mcu_driver_diagnostics_     = NO_ERROR;
  mc_driver_diagnostics_      = NO_ERROR;
  urg_node_diagnostics_       = NO_ERROR;
  camera_left_diagnostics_    = NO_ERROR;
  camera_right_diagnostics_   = NO_ERROR;
  sensor_safety_diagnostics_   = NO_ERROR;
  global_diagnostics_ = NO_ERROR;
}

/**
 * @name        GetParamSettings
 * @brief       Sets parameter data members according to values from ROS param server
 */
void DiagnosticsMessenger::GetParamSettings()
{
  ROS_INFO_STREAM("*** Diagnostics Messenger getting Parameters ***");
  GetParamUtil::GetParam("/sm_properties/enable_diagnostics_monitor", enable_diagnostics_monitor_, false);
  ros_->manager_->SetEnableDiagnostics(enable_diagnostics_monitor_);
  GetParamUtil::GetParam("/sm_properties/diagnostics_timeout", diagnostics_timeout_, 10.0);
  ros_->manager_->SetDiagnosticsTimeout(diagnostics_timeout_);
}

/**
 * @name  InitializePublishersAndSubscribers
 * @brief Manages publishers and subscribers
 */
void DiagnosticsMessenger::InitializePublishersAndSubscribers()
{
  ROS_INFO_STREAM("*** Diagnostics Messenger Initializing subs and pubs ***");
  diagnostics_sub_    = node_.subscribe(avidbots_topics::diagnostics_global_topic, 1,
                        &DiagnosticsMessenger::DiagnosticStatusCallBack, this);
}

/**
 * @name  InitializeTimers
 * @brief Manages Timers
 */
void DiagnosticsMessenger::InitializeTimers()
{
  ROS_INFO_STREAM("*** Diagnostics Messenger Initializing timers ***");
  /* Create diagnostics timer */
  diagnostics_timer_ = node_.createTimer
      (ros::Duration(0.1), &DiagnosticsMessenger::DiagnosticsTimerCallBack, this);
}

/**
 * @name    DiagnosticsTimerCallBack()
 * @brief   The diagnostics callback function that updates the state
 *          transition diagnostics variable.
 */
void DiagnosticsMessenger::DiagnosticsTimerCallBack(const ros::TimerEvent &)
{
  if (ros_->manager_->GetEnableSafetyMonitor() && ros_->manager_->GetEnableDiagnostics())
  {
    ros_->manager_->SetDiagnosticsStatus(diagnostics_status_);
    if ((diagnostics_status_ == ERROR) && (ros_->manager_->GetEStopStatus() == NO_ERROR))
    {
      ros_->manager_->SMStateEvent(DIAGNOSTICS_EVENT);
    }
  }
}

/**
 * @name  DiagnosticStatusCallBack
 * @brief Call back for the diagnostics aggregator top level state
 * @param[in] diagnostics_msg: The diagnostics msg that is created by
 *                             diagnostics aggregator.
 */
void DiagnosticsMessenger::DiagnosticStatusCallBack(const avidbots_msgs::diagnostics &diagnostics)
{
  mcu_driver_diagnostics_ = diagnostics.status_mcu ? ERROR : NO_ERROR;
  mc_driver_diagnostics_ = diagnostics.status_mc ? ERROR : NO_ERROR;
  urg_node_diagnostics_ = diagnostics.status_urg ? ERROR : NO_ERROR;
  camera_left_diagnostics_ = diagnostics.status_camera_left ? ERROR : NO_ERROR;
  camera_right_diagnostics_ = diagnostics.status_camera_right ? ERROR : NO_ERROR;
  sensor_safety_diagnostics_ = diagnostics.status_sensor_safety ? ERROR : NO_ERROR;
  global_diagnostics_ = diagnostics.status_global ? ERROR : NO_ERROR;

  if (global_diagnostics_ == ERROR)
  {
    diagnostics_status_ = ERROR;
    diagnostics_description_ = diagnostics.status_description;
  }
  else
  {
    diagnostics_status_ = NO_ERROR;
    diagnostics_description_ = "Diagnostics OK";
  }

  ros_->manager_->SetDiagnosticsDescription(diagnostics_description_);
}

/**
 * @name    StopDiagnosticsTimer()
 * @brief   Stops the diagnostics timer
 */
void DiagnosticsMessenger::StopDiagnosticsTimer()
{
  diagnostics_timer_.stop();
}

/**
 * @name    StartDiagnosticsTimer()
 * @brief   Starts the diagnostics timer
 */
void DiagnosticsMessenger::StartDiagnosticsTimer()
{
  diagnostics_timer_.start();
}
