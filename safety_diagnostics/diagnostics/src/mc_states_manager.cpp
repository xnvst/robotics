/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	mc_states_manager.cpp
 * @brief	Source File containing the MCStatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/mc_states_manager.h"

/**
 * @name 	MCStatesManager
 * @brief	Default constructor
 */
MCStatesManager::MCStatesManager()
{
}

/**
 * @name  ~MCStatesManager
 * @brief Destructor
 */
MCStatesManager::~MCStatesManager() {}

/**
 * @name        Init()
 * @brief       Initialize MCStatesManager
 * @param[in]   hardware_id, to identify different device
 */
void MCStatesManager::Init(uint8_t hardware_id)
{
  ROS_INFO_STREAM("MCStatesManager Init");
  id_ = hardware_id;
  num_states_ = avidbots_diagnostics_constants::MC_MSG_ID_NUM;
  states_ = avidbots_msgs::diagnostics_states::kStateOk;
}

/**
 * @name        UpdateDiagnostics()
 * @brief       Update Diagnostics message
 * @param[in]   states, data reference for avidbots_msgs::diagnostics_states
 * @param[in]   diagnostics_msg, data reference for avidbots_msgs::diagnostics
 */
void MCStatesManager::UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                        avidbots_msgs::diagnostics &diagnostics_msg)
{
  diagnostics_msg.hardware_id = avidbots_msgs::diagnostics_states::kMCDriver;
  diagnostics_msg.name = "MC:\n";
  diagnostics_msg.status_mc = states.states;
  diagnostics_msg.stamp = ros::Time::now();
}
