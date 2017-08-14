/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	camera_states_manager.cpp
 * @brief	Source File containing the CameraStatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/camera_states_manager.h"

/**
 * @name 	CameraStatesManager
 * @brief	Default constructor
 */
CameraStatesManager::CameraStatesManager()
{
}

/**
 * @name  ~CameraStatesManager
 * @brief Destructor
 */
CameraStatesManager::~CameraStatesManager() {}

/**
 * @name        Init()
 * @brief       Initialize CameraStatesManager
 * @param[in]   hardware_id, to identify different device 
 */
void CameraStatesManager::Init(uint8_t hardware_id)
{
  ROS_INFO_STREAM("CameraStatesManager Init");
  id_ = hardware_id;
  num_states_ = avidbots_diagnostics_constants::CAMERA_MSG_ID_NUM;
  states_ = avidbots_msgs::diagnostics_states::kStateOk;
}

/**
 * @name        UpdateDiagnostics()
 * @brief       Update Diagnostics message
 * @param[in]   states, data reference for avidbots_msgs::diagnostics_states
 * @param[in]   diagnostics_msg, data reference for avidbots_msgs::diagnostics
 */
void CameraStatesManager::UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                            avidbots_msgs::diagnostics &diagnostics_msg)
{
  if (states.hardware_id == avidbots_msgs::diagnostics_states::k3DCameraLeft)
  {
    diagnostics_msg.hardware_id = avidbots_msgs::diagnostics_states::k3DCameraLeft;
    diagnostics_msg.name = "Left Camera:\n";
    diagnostics_msg.status_camera_left = states.states;
  }
  else if (states.hardware_id == avidbots_msgs::diagnostics_states::k3DCameraRight)
  {
    diagnostics_msg.hardware_id = avidbots_msgs::diagnostics_states::k3DCameraRight;
    diagnostics_msg.name = "Right Camera:\n";
    diagnostics_msg.status_camera_right = states.states;
  }
  else
  {
    return;
  }
  diagnostics_msg.stamp = states.stamp;
}
