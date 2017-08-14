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
 * @brief   Implementation file for ExpExpSafetyZoneState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/exp_safetyzone_state.h>
#include <avidbots_safety_monitor/states/dangerzone_state.h>

/**
 * @name        ExpSafetyZpneState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
ExpSafetyZoneState::ExpSafetyZoneState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void ExpSafetyZoneState::Initialize()
{
  ROS_INFO("************Initialize EXP_SAFETYZONE_STATE***************");
  state_msg_.data = "ExpSafetyZoneState";
  Execute();
}

/**
 * @name        Execute
 * @brief       Execute State
 */
void ExpSafetyZoneState::Execute()
{
  ROS_DEBUG("************Execute EXP_SAFETYZONE_STATE***************");
  manager_->PublishState(state_msg_);
  manager_->PublishZeroVelocity();
}

