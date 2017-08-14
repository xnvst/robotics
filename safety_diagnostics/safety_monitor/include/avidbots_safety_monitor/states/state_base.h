/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    state_base
 * @brief   Header file for StateBase class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_STATES_STATE_BASE_H
#define AVIDBOTS_SAFETY_MONITOR_STATES_STATE_BASE_H

// CPP
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <avidbots_msgs/sm_status.h>

// LOCAL
#include "avidbots_library/safety_zone/properties.h"
#include "avidbots_safety_monitor/sm_manager.h"

class SMManager;

/**
 * @name    StateBase
 * @brief   Base object for each state.
 */
class StateBase
{
public:
  explicit StateBase(SMManager* manager);
  virtual ~StateBase() {}

  // Action functions that result in a state transition
  virtual void Initialize() {}
  virtual void Execute() {}

protected:
  SMManager*      manager_;
  std_msgs::String state_msg_;
  avidbots_msgs::sm_status msg;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_STATES_STATE_BASE_H
