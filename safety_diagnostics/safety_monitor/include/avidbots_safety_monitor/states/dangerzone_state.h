/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    dangerzone_state
 * @brief   Header file for DangerZoneState class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_STATES_DANGERZONE_STATE_H
#define AVIDBOTS_SAFETY_MONITOR_STATES_DANGERZONE_STATE_H

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/sm_manager.h"
#include "avidbots_safety_monitor/states/state_base.h"

class SMManager;

/**
 * @name    DangerZoneState
 * @brief   idle state object
 */
class DangerZoneState : public StateBase
{
public:
  explicit DangerZoneState(SMManager* manager);
  ~DangerZoneState() {}

  void Initialize();
  void Execute();

private:
};

#endif  // AVIDBOTS_SAFETY_MONITOR_STATES_DANGERZONE_STATE_H
