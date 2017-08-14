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
 * @brief   Implementation file for EstopState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/estop_state.h>

/**
 * @name        EStopState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
EStopState::EStopState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void EStopState::Initialize()
{
  ROS_INFO("************Initialize ESTOP_STATE***************");
  state_msg_.data = "EStopState";
  Execute();
}

/**
 * @name        Execute
 * @brief       Execute function
 */
void EStopState::Execute()
{
  ROS_DEBUG("************Execute ESTOP_STATE***************");
  manager_->PublishState(state_msg_);
  manager_->PublishZeroVelocity();
  manager_->PublishDangerZoneObstacle(!manager_->GetDangerZoneStatus());
}
