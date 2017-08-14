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
 * @brief   Implementation file for LowVoltageState class
 * @author  Keshav Iyengar, Feng Cao
 */

// ROS
#include <ros/console.h>

// LOCAL
#include <avidbots_safety_monitor/states/low_voltage_state.h>

/**
 * @name        LowVoltageState
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
LowVoltageState::LowVoltageState(SMManager* manager): StateBase(manager)
{
}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void LowVoltageState::Initialize()
{
  ROS_INFO("************Initialize LOW_VOLTAGE_STATE***************");
  state_msg_.data = "LowVoltageState";
  manager_->RestartFailureStopWatch();
  Execute();
}

/**
 * @name        Execute
 * @brief       Execute function
 */
void LowVoltageState::Execute()
{
  ROS_DEBUG("************Execute LOW_VOLTAGE_STATE***************");
  manager_->PublishState(state_msg_);
  manager_->PublishZeroVelocity();
  if (manager_->GetFailureStopWatchElapsed() > ecl::TimeStamp(manager_->GetDiagnosticsTimeout()))
  {
    msg.diagnostics_status = true;
    msg.low_voltage_status = (manager_->GetLowVoltageStatus() == NO_ERROR) ? true : false;
    msg.diagnostics_description = manager_->GetDiagnosticsDescription();
    manager_->PublishSMStatus(msg);
  }
}

