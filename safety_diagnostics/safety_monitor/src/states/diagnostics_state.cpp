/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    diagnostics_state
 * @brief   Implementation file for DiagnosticsState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/diagnostics_state.h>

/**
 * @name        DiagnosticsState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
DiagnosticsState::DiagnosticsState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void DiagnosticsState::Initialize()
{
  ROS_INFO("************Initialize DIAGNOSTICS_STATE***************");
  state_msg_.data = "DiagnosticsState";
  manager_->RestartFailureStopWatch();
  Execute();
}

/**
 * @name        Execute
 * @brief       Execute function
 */
void DiagnosticsState::Execute()
{
  ROS_DEBUG("************Execute DIAGNOSTICS_STATE***************");
  manager_->PublishState(state_msg_);
  manager_->PublishZeroVelocity();
  if (manager_->GetFailureStopWatchElapsed() > ecl::TimeStamp(DIAGNOSTICS_TIMEOUT))
  {
    msg.diagnostics_status = (manager_->GetDiagnosticsStatus() == NO_ERROR) ? true : false;
    msg.low_voltage_status = true;
    msg.diagnostics_description = manager_->GetDiagnosticsDescription();
    manager_->PublishSMStatus(msg);
  }
}

