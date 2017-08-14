/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    idle_state
 * @brief   Implementation file for IdleState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/idle_state.h>

/**
 * @name        IdleState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
IdleState::IdleState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void IdleState::Initialize()
{
  ROS_INFO("************Initialize IDLE_STATE***************");
  state_msg_.data = "IdleState";
  manager_->RestartFailureStopWatch();
  Execute();
}

/**
 * @name        Execute
 * @brief       Execute function
 */
void IdleState::Execute()
{
  ROS_DEBUG("************Execute IDLE_STATE***************");
  manager_->PublishState(state_msg_);
  manager_->SetSmFailureReset(false);
  msg.diagnostics_status = (manager_->GetDiagnosticsStatus() == NO_ERROR) ? true : false;
  msg.low_voltage_status = (manager_->GetLowVoltageStatus() == NO_ERROR) ? true : false;
  msg.diagnostics_description = manager_->GetDiagnosticsDescription();
  manager_->PublishSMStatus(msg);
  manager_->PublishDangerZoneObstacle(!manager_->GetDangerZoneStatus());
}
