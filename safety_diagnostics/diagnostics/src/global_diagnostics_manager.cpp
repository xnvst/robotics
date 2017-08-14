/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	global_diagnostics_manager.cpp
 * @brief	Source File containing the StatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/global_diagnostics_manager.h"

/**
 * @name 	GlobalDiagnosticsManager
 * @brief	Default constructor
 */
GlobalDiagnosticsManager::GlobalDiagnosticsManager()
{
}

/**
 * @name  ~GlobalDiagnosticsManager
 * @brief Destructor
 */
GlobalDiagnosticsManager::~GlobalDiagnosticsManager() {}

/**
 * @name  AddQueue
 * @brief Add Item to Queue
 */
bool GlobalDiagnosticsManager::AddQueue(const avidbots_msgs::diagnostics_states &states)
{
  bool ret = false;
  ROS_INFO("GlobalDiagnosticsManager::AddQueue %d\n", states.hardware_id);
  ret = diagnostics_states_queue_.push(states);
  return ret;
}

/**
 * @name  ReadQueue
 * @brief Read Item from Queue
 */
bool GlobalDiagnosticsManager::ReadQueue(avidbots_msgs::diagnostics_states &states)
{
  bool ret = diagnostics_states_queue_.pop(states);
  if (ret)
  {
    ROS_INFO("GlobalDiagnosticsManager::ReadQueue %d\n", states.hardware_id);
  }
  return ret;
}
