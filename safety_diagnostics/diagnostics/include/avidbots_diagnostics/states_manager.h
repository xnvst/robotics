/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	states_manager.h
 * @brief	Header file containing the StatesManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_DIAGNOSTICS_STATES_MANAGER_H
#define AVIDBOTS_DIAGNOSTICS_STATES_MANAGER_H

// CPP
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>

// LCOAL
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/diagnostics_states.h>
#include <avidbots_msgs/diagnostics.h>
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"

class StatesManager
{
public:
  virtual ~StatesManager();
  StatesManager();
  virtual void Init(uint8_t hardware_id) = 0;
  virtual void UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                 avidbots_msgs::diagnostics &diagnostics_msg) = 0;

protected:
  uint8_t id_;
  uint8_t num_states_;
  void UpgradeState(uint8_t value, uint8_t &status);
};

#endif  // AVIDBOTS_DIAGNOSTICS_STATES_MANAGER_H
