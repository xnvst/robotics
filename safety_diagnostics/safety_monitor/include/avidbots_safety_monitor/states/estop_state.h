/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    estop_state
 * @brief   Header file for EStopState class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_STATES_ESTOP_STATE_H
#define AVIDBOTS_SAFETY_MONITOR_STATES_ESTOP_STATE_H

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/sm_manager.h"
#include "avidbots_safety_monitor/states/state_base.h"

class SMManager;

/**
 * @name    EStopState
 * @brief   idle state object
 */
class EStopState : public StateBase
{
public:
  explicit EStopState(SMManager* manager);
  ~EStopState() {}

  void Initialize();
  void Execute();

private:
};

#endif  // AVIDBOTS_SAFETY_MONITOR_STATES_ESTOP_STATE_H
