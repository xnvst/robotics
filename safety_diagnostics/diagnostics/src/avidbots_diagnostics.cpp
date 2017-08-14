/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_diagnostics.cpp
 * @brief	Source File containing the AvidbotsDiagnostics class
 * @author	Feng Cao
 */

#include <string>
#include "avidbots_diagnostics/avidbots_diagnostics.h"

/**
 * @name 	AvidbotsDiagnostics
 * @brief	Default constructor
 */
AvidbotsDiagnostics::AvidbotsDiagnostics() : global_enable_(false)
{
  diagnostics_msg_.status_mcu = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_mc = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_urg = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_camera_left = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_camera_right = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_sensor_safety = avidbots_msgs::diagnostics_states::kStateOk;
  diagnostics_msg_.status_global = avidbots_msgs::diagnostics_states::kStateOk;
}

/**
 * @name  ~AvidbotsDiagnostics
 * @brief Destructor
 */
AvidbotsDiagnostics::~AvidbotsDiagnostics() {}

/**
 * @name        Init()
 * @brief       Initialize ros interface
 * @param[in]   private_nh, node handler
 * @param[in]   node_handle, node handler
 */
void AvidbotsDiagnostics::Init(const ros::NodeHandle& private_nh,
                                     ros::NodeHandle *node_handle)
{
  ROS_INFO_STREAM("AvidbotsDiagnostics Init");

  node_handle_        = node_handle;

  // Initialize publishers and subscribers
  ManagePubAndSub();

  // Get parameters
  GetParamSettings(private_nh);

  // Initialize States Manager
  AvidbotsDiagnosticsInstance.mcu_states_mgr_.Init(avidbots_msgs::diagnostics_states::kMCUDriver);
  AvidbotsDiagnosticsInstance.mc_states_mgr_.Init(avidbots_msgs::diagnostics_states::kMCDriver);
  AvidbotsDiagnosticsInstance.urg_states_mgr_.Init(avidbots_msgs::diagnostics_states::kURGNode);
  AvidbotsDiagnosticsInstance.camera_left_states_mgr_.Init(avidbots_msgs::diagnostics_states::k3DCameraLeft);
  AvidbotsDiagnosticsInstance.camera_right_states_mgr_.Init(avidbots_msgs::diagnostics_states::k3DCameraRight);
  AvidbotsDiagnosticsInstance.sensor_safety_states_mgr_.Init(avidbots_msgs::diagnostics_states::kSensorSafety);
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 * @param[in]   private_nh, node handler
 */
void AvidbotsDiagnostics::GetParamSettings
            (const ros::NodeHandle& private_nh)
{
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void AvidbotsDiagnostics::ManagePubAndSub()
{
  // subscribe
  ROS_INFO_STREAM("*** Subscribing for diagnostics states topic ***");
  diagnostics_states_sub_    = node_handle_->
                      subscribe(avidbots_topics::diagnostics_states_topic, 1,
                      &AvidbotsDiagnostics::DiagnosticsStatesCallback, this);

  urg_diagnostics_states_sub_    = node_handle_->
                      subscribe(avidbots_topics::ros_diagnostics_status_topic, 1,
                      &AvidbotsDiagnostics::URGDiagnosticsStatesCallback, this);

  camera_left_diagnostics_states_sub_    = node_handle_->
                      subscribe("/avidbots/diagnostics/camera_left", 1,
                      &AvidbotsDiagnostics::CameraLeftDiagnosticsStatesCallback, this);

  camera_right_diagnostics_states_sub_    = node_handle_->
                      subscribe("/avidbots/diagnostics/camera_right", 1,
                      &AvidbotsDiagnostics::CameraRightDiagnosticsStatesCallback, this);

  sensor_toggle_sub_       = node_handle_->
                      subscribe("/avidbots/ss/sensor_status", 1,
                      &AvidbotsDiagnostics::SensorToggleCallback, this);

  // publish
  ROS_INFO_STREAM("*** Publishing diagnostics global topic ***");
  diagnostics_global_pub_        = node_handle_->advertise
                    <avidbots_msgs::diagnostics>
                    (avidbots_topics::diagnostics_global_topic, 1, true);
}

/**
 * @name    PublishDiagnosticsUpdate
 * @brief   Publish Diagnostics Update
*/
void AvidbotsDiagnostics::PublishDiagnosticsGlobalUpdate()
{
  if (!global_enable_)
  {
    diagnostics_msg_.status_mcu = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_mc = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_urg = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_camera_left = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_camera_right = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_sensor_safety = avidbots_msgs::diagnostics_states::kStateOk;
    diagnostics_msg_.status_global = avidbots_msgs::diagnostics_states::kStateOk;
  }
  if ((diagnostics_msg_.status_mcu != avidbots_msgs::diagnostics_states::kStateOk)
      || (diagnostics_msg_.status_mc != avidbots_msgs::diagnostics_states::kStateOk)
      || (diagnostics_msg_.status_urg != avidbots_msgs::diagnostics_states::kStateOk)
      || (diagnostics_msg_.status_camera_left != avidbots_msgs::diagnostics_states::kStateOk)
      || (diagnostics_msg_.status_camera_right != avidbots_msgs::diagnostics_states::kStateOk)
      || (diagnostics_msg_.status_sensor_safety != avidbots_msgs::diagnostics_states::kStateOk))
  {
    diagnostics_msg_.status_global = avidbots_msgs::diagnostics_states::kStateError;
  }
  else
  {
    diagnostics_msg_.status_global = avidbots_msgs::diagnostics_states::kStateOk;
  }
  diagnostics_global_pub_.publish(diagnostics_msg_);
}

/**
 * @name    DiagnosticsStatesCallback
 * @brief   Diagnostics States Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsDiagnostics::DiagnosticsStatesCallback
                   (const avidbots_msgs::diagnostics_statesPtr &diagnostics_states)
{
  if (!global_enable_)
  {
    return;
  }
  avidbots_msgs::diagnostics_states states = *diagnostics_states;
  switch (diagnostics_states->hardware_id)
  {
    case avidbots_msgs::diagnostics_states::kMCUDriver:
      AvidbotsDiagnosticsInstance.mcu_states_mgr_.UpdateDiagnostics(states, diagnostics_msg_);
      break;
    case avidbots_msgs::diagnostics_states::kMCDriver:
      AvidbotsDiagnosticsInstance.mc_states_mgr_.UpdateDiagnostics(states, diagnostics_msg_);
      break;
    case avidbots_msgs::diagnostics_states::kURGNode:
      // urg_node is treated seperately to re-use its built-in diagnostics message
      break;
    case avidbots_msgs::diagnostics_states::k3DCameraLeft:
    case avidbots_msgs::diagnostics_states::k3DCameraRight:
      // camera is disabled
      break;
    case avidbots_msgs::diagnostics_states::kSensorSafety:
      AvidbotsDiagnosticsInstance.sensor_safety_states_mgr_.UpdateDiagnostics(states, diagnostics_msg_);
      break;
    default:
      return;
  }
  diagnostics_msg_.status_description = diagnostics_states->description;
}

/**
 * @name    URGDiagnosticsStatesCallback
 * @brief   URG Diagnostics States Callback
 * @param[in]   urg_diagnostics_array, const msg pointer for diagnostic_msgs::DiagnosticArrayConstPtr
 */
void AvidbotsDiagnostics::URGDiagnosticsStatesCallback
                   (const diagnostic_msgs::DiagnosticArrayConstPtr &urg_diagnostics_array)
{
  // ignore the laser published diagnostics check due to content not in control
  // laser diagnostics including scan frequency and data integrity is covered by sensor safety
  return;

  if (!global_enable_)
  {
    return;
  }
  diagnostic_msgs::DiagnosticStatus urg_diagnostics_status;
  ROS_DEBUG_STREAM("URGDiagnosticsStatesCallback size=" << urg_diagnostics_array->status.size() << std::endl);
  avidbots_msgs::diagnostics_statesPtr urg_diagnostics_states;
  urg_diagnostics_states.reset(new avidbots_msgs::diagnostics_states);
  std::size_t found;
  std::string urg_str("urg_node");
  std::string hokuyo_str("hokuyo_node");
  for (int i = 0; i < urg_diagnostics_array->status.size(); i++)
  {
    urg_diagnostics_status = urg_diagnostics_array->status[i];
    found = urg_diagnostics_status.name.find(urg_str);
    if (found != std::string::npos)
    {
      AvidbotsDiagnosticsInstance.urg_states_mgr_.SetLaserType(avidbots_diagnostics_constants::LASER_URG);
      ROS_DEBUG("URGDiagnosticsStatesCallback: %s, %d\n",
               urg_diagnostics_status.name.c_str(), urg_diagnostics_status.level);
      urg_diagnostics_states->states = urg_diagnostics_status.level;
    }
    else
    {
      found = urg_diagnostics_status.name.find(hokuyo_str);
      if (found != std::string::npos)
      {
        AvidbotsDiagnosticsInstance.urg_states_mgr_.SetLaserType(avidbots_diagnostics_constants::LASER_HOKUYO);
        ROS_DEBUG("URGDiagnosticsStatesCallback: %s, %d\n",
               urg_diagnostics_status.name.c_str(), urg_diagnostics_status.level);
        urg_diagnostics_states->states = urg_diagnostics_status.level;
      }
    }
  }
  if (found != std::string::npos && urg_diagnostics_array->status.size())
  {
    urg_diagnostics_states->stamp = ros::Time::now();
    urg_diagnostics_states->hardware_id = avidbots_msgs::diagnostics_states::kURGNode;
    AvidbotsDiagnosticsInstance.global_diagnostics_mgr_.AddQueue(*urg_diagnostics_states);
  }
  urg_diagnostics_states.reset();
}

/**
 * @name    CameraLeftDiagnosticsStatesCallback
 * @brief   Camera left Diagnostics States Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsDiagnostics::CameraLeftDiagnosticsStatesCallback
                   (const avidbots_msgs::diagnostics_statesPtr &diagnostics_states)
{
  return;   // disable camera diagnostics in release 1.3
}

/**
 * @name    CameraRightDiagnosticsStatesCallback
 * @brief   Camera Right Diagnostics States Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsDiagnostics::CameraRightDiagnosticsStatesCallback
                   (const avidbots_msgs::diagnostics_statesPtr &diagnostics_states)
{
  return;   // disable camera diagnostics in release 1.3
}

/**
 * @name  SensorToggleCallback
 * @brief Call back for the sensor toggle msg.
 * @param[in] avidbots_msgs::sensor_toggle
 */
void AvidbotsDiagnostics::SensorToggleCallback
                   (const avidbots_msgs::sensor_toggle &sensor_toggle)
{
  if (sensor_toggle.sensor_id == avidbots_msgs::sensor_toggle::kGlobal)
  {
    global_enable_ = sensor_toggle.status;
  }
}
