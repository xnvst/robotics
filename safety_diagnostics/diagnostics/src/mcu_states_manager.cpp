/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	mcu_states_manager.cpp
 * @brief	Source File containing the MCUStatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/mcu_states_manager.h"
#include "avidbots_library/socket_can/avidbots_can_open_constants.h"

/**
 * @name 	MCUStatesManager
 * @brief	Default constructor
 */
MCUStatesManager::MCUStatesManager()
{
}

/**
 * @name  ~MCUStatesManager
 * @brief Destructor
 */
MCUStatesManager::~MCUStatesManager() {}

/**
 * @name        Init()
 * @brief       Initialize MCUStatesManager
 * @param[in]   hardware_id, to identify different device
 */
void MCUStatesManager::Init(uint8_t hardware_id)
{
  ROS_INFO_STREAM("MCUStatesManager Init");
  id_ = hardware_id;
  num_states_ = avidbots_diagnostics_constants::MCU_MSG_ID_NUM;
  states_ = avidbots_msgs::diagnostics_states::kStateOk;
}

/**
 * @name        UpdateDiagnostics()
 * @brief       Update Diagnostics message
 * @param[in]   states, data reference for avidbots_msgs::diagnostics_states
 * @param[in]   diagnostics_msg, data reference for avidbots_msgs::diagnostics
 */
void MCUStatesManager::UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                         avidbots_msgs::diagnostics &diagnostics_msg)
{
  diagnostics_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCUDriver;
  diagnostics_msg.name = "MCU:\n";
  diagnostics_msg.status_mcu = states.states;
  diagnostics_msg.stamp = ros::Time::now();
}
