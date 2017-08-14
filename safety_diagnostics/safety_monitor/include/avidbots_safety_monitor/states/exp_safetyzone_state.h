/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    exp_safetyzone_state
 * @brief   Header file for ExpDangerZoneState class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_STATES_EXP_SAFETYZONE_STATE_H
#define AVIDBOTS_SAFETY_MONITOR_STATES_EXP_SAFETYZONE_STATE_H

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/sm_manager.h"
#include "avidbots_safety_monitor/states/state_base.h"

class SMManager;

/**
 * @name    ExpDangerZoneState
 * @brief   Exponential safetyzone state object
 */
class ExpSafetyZoneState : public StateBase
{
public:
  explicit ExpSafetyZoneState(SMManager* manager);
  ~ExpSafetyZoneState() {}

  /* Action Functions */
  void Initialize();
  void Execute();

private:
};

#endif  // AVIDBOTS_SAFETY_MONITOR_STATES_EXP_SAFETYZONE_STATE_H
