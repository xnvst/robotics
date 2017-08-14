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
 * @brief   Implementation file for DangerZoneState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/dangerzone_state.h>

/**
 * @name        DangerZoneState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
DangerZoneState::DangerZoneState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void DangerZoneState::Initialize()
{
  ROS_INFO("************Initialize DANGERZONE_STATE***************");
  state_msg_.data = "DangerzoneState";
  manager_->RestartFailureStopWatch();
  Execute();
}


/**
 * @name        Execute
 * @brief       Exeuction function
 */
void DangerZoneState::Execute()
{
  ROS_DEBUG("************Execute DANGERZONE_STATE***************");
  manager_->PublishState(state_msg_);

  /* Trigger estop or publish zero velocity */
  if (manager_->GetDangerZoneZeroVelEnabled())
  {
    manager_->PublishZeroVelocity();
  }
  else
  {
    manager_->PublishMCUOutputCommand();
  }

  /* Timeout to avoid temporary obstacles */
  if (manager_->GetFailureStopWatchElapsed() > ecl::TimeStamp(manager_->GetObstacleTimeout()))
  {
    manager_->PublishDangerZoneObstacle(!manager_->GetDangerZoneStatus());
  }
}
