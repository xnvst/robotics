/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    sm_manager
 * @brief   Implementation file for SMManager class
 * @author  Keshav Iyengar, Feng Cao
 */

// CPP
#include <string>

// ROS
#include <ros/console.h>
#include <ros/package.h>

// LOCAL
#include "avidbots_safety_monitor/sm_manager.h"
#include <avidbots_safety_monitor/states/idle_state.h>
#include <avidbots_safety_monitor/states/diagnostics_state.h>
#include <avidbots_safety_monitor/states/dangerzone_state.h>
#include <avidbots_safety_monitor/states/exp_safetyzone_state.h>
#include <avidbots_safety_monitor/states/low_voltage_state.h>
#include <avidbots_safety_monitor/states/estop_state.h>

/**
 * @name        SMManager
 * @brief       Constructor.
 */
SMManager::SMManager() : ros_(new ROSInterface(this))
{
  // Initialize statuses
  ROS_INFO("*** Initializing Safety Monitor ***");
  /* Initialize State Transition Variables */
  sm_manager_enable_safety_monitor_       = NO_ERROR;
  sm_manager_diagnostics_status_          = NO_ERROR;
  sm_manager_exp_safetyzone_status_       = NO_ERROR;
  sm_manager_dangerzone_status_           = NO_ERROR;
  sm_manager_low_voltage_status_          = NO_ERROR;
  sm_manager_estop_status_                = NO_ERROR;

  state_ = NULL;
  prev_state_ = INIT_STATE;
  curr_state_ = INIT_STATE;
  next_state_ = INIT_STATE;
  curr_event_ = INIT_EVENT;
}

/**
 * @name        SMManager
 * @brief       Destructor.
 */
SMManager::~SMManager()
{
  delete state_;
  delete ros_;
}

/**
 * @name        StateTransition
 * @brief       transition current state
 * @param[in]   state: next state
 */
void SMManager::StateTransition(StateBase* state)
{
  StateBase* tmp = state_;
  state_ = state;
  if (tmp) delete tmp;
  state_->Initialize();
}

/**
 * @name        Initialize
 * @brief       function to Initialize
 */
void SMManager::Initialize()
{
  ros_->Initialize();
  next_state_ = IDLE_STATE;
}

/**
 * @name        Update
 * @brief       Function to execute the state in a timed manner
 */
void SMManager::Update()
{
  if (curr_state_ == INIT_STATE)
  {
    if (next_state_ != INIT_STATE)
    {
      SetState(next_state_);
    }
    else
    {
      return;
    }
  }
  if (curr_state_ != next_state_)
  {
    SetState(next_state_);
  }
  switch (curr_state_)
  {
    case IDLE_STATE:
      if (GetUIState() == STATE_DANGER_OBSTACLE)
      {
        next_state_ = DANGERZONE_STATE;
      }
      if (GetUIState() == STATE_SM_FAILURE)
      {
        PublishZeroVelocity();
        if (GetFailureStopWatchElapsed() > ecl::TimeStamp(DIAGNOSTICS_TIMEOUT))
        {
          next_state_ = DIAGNOSTICS_STATE;
        }
      }
      break;
    case DIAGNOSTICS_STATE:
      if (GetFailureStopWatchElapsed() > ecl::TimeStamp(DIAGNOSTICS_TIMEOUT))
      {
        if (GetUIState() == STATE_SM_FAILURE || GetUIState() == STATE_DIAGNOSTICS)
        {
          if ((GetDiagnosticsStatus() == NO_ERROR) && GetSmFailureReset())
          {
            next_state_ = IDLE_STATE;
          }
        }
        else
        {
          if (GetDiagnosticsStatus() == NO_ERROR)
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      break;
    case DANGERZONE_STATE:
      if (GetFailureStopWatchElapsed() > ecl::TimeStamp(GetObstacleTimeout()))
      {
        if (GetUIState() == STATE_DANGER_OBSTACLE  || GetUIState() == STATE_DIAGNOSTICS)
        {
          // If the disable reset in sm properties is set, we automatically reset
          if ((GetDangerZoneStatus() == NO_ERROR) && (GetSkipDangerObstacleReset() == NO_ERROR))
          {
            next_state_ = IDLE_STATE;
          }
        }
        else
        {
          if ((GetDangerZoneStatus() == NO_ERROR))
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      break;
    case EXP_SAFETYZONE_STATE:
      if (GetExpSafetyZoneStatus() == NO_ERROR)
      {
        next_state_ = IDLE_STATE;
      }
      if (GetDangerZoneStatus() == ERROR)
      {
        next_state_ = DANGERZONE_STATE;
      }
      break;
    case LOW_VOLTAGE_STATE:
      if (GetFailureStopWatchElapsed() > ecl::TimeStamp(GetDiagnosticsTimeout()))
      {
        if (GetUIState() == STATE_SM_FAILURE  || GetUIState() == STATE_DIAGNOSTICS)
        {
          if ((GetLowVoltageStatus() == NO_ERROR) && GetSmFailureReset())
          {
            next_state_ = IDLE_STATE;
          }
        }
        else
        {
          if (GetLowVoltageStatus() == NO_ERROR)
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      break;
    case ESTOP_STATE:
      if (GetEStopStatus() == NO_ERROR)
      {
        if (prev_state_ == DANGERZONE_STATE)
        {
          next_state_ = prev_state_;
        }
        else
        {
          next_state_ = IDLE_STATE;
        }
      }
      break;
    default:
      break;
  }
  state_->Execute();
}

/////////////////////
/* Event Functions */
/////////////////////

/**
 * @name        SMStateEvent
 * @brief       Safety Monitor state event from ROS Interface
 */
void SMManager::SMStateEvent(Events event)
{
  switch (event)
  {
    case DIAGNOSTICS_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: DiagnosticsStatusEvent");
      if (curr_state_ == IDLE_STATE)
      {
        next_state_ = DIAGNOSTICS_STATE;
      }
      else if (curr_state_ == ESTOP_STATE)
      {
        if ((GetEStopStatus() == NO_ERROR) && (GetDiagnosticsStatus() == NO_ERROR))
        {
          ROS_WARN("Safety Monitor EstopError State cleared!");
          next_state_ = IDLE_STATE;
        }
      }
      break;
    case DANGERZONE_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: DangerZoneStatusEvent");
      if (curr_state_ == IDLE_STATE)
      {
        if ((GetDangerZoneStatus() == ERROR) && (GetLocalControllerSafetyStatus() == NO_ERROR))
        {
          next_state_ = DANGERZONE_STATE;
        }
      }
      else if (curr_state_ == EXP_SAFETYZONE_STATE)
      {
        if (GetExpSafetyZoneStatus() == ERROR)
        {
          next_state_ = DANGERZONE_STATE;
        }
      }
      break;
    case EXP_SAFETYZONE_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: ExpSafetyZoneStatusEvent");
      if (curr_state_ == IDLE_STATE)
      {
        if (GetExpSafetyZoneStatus() == ERROR)
        {
          next_state_ = EXP_SAFETYZONE_STATE;
        }
      }
      break;
    case LOW_VOLTAGE_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: LowVoltageStatusEvent");
      if (curr_state_ == IDLE_STATE)
      {
        next_state_ = LOW_VOLTAGE_STATE;
      }
      break;
    case ESTOP_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: EStopStatusEvent");
      if (GetEStopStatus() == ERROR)
      {
        next_state_ = ESTOP_STATE;
      }
      break;
    case FAILURE_RESET_EVENT:
      if (curr_event_ != event) ROS_INFO("SMManager: ResetStatusEvent");
      if (curr_state_ == DIAGNOSTICS_STATE)
      {
        if (GetDiagnosticsStatus() == NO_ERROR)
        {
          ROS_WARN("Safety Monitor DiagnosticsError State cleared!");
          if (GetSmFailureReset())
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      else if (curr_state_ == LOW_VOLTAGE_STATE)
      {
        if (GetLowVoltageStatus() == NO_ERROR)
        {
          ROS_WARN("Safety Monitor LowVoltageError State cleared!");
          if (GetSmFailureReset())
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      else if (curr_state_ == DANGERZONE_STATE)
      {
        if (GetDangerZoneStatus() == NO_ERROR)
        {
          if (GetSmFailureReset())
          {
            next_state_ = IDLE_STATE;
          }
        }
      }
      break;
    default:
      break;
  }
  curr_event_ = event;
}


//////////////////////
/* Setter Functions */
//////////////////////

/**
 * @name        SetState()
 * @brief       Sets the enum of state
 * @param[in]   state: an enum (States) to set the state to
 */
void SMManager::SetState(States state)
{
  ROS_INFO("*** SetState transition from %d to %d", curr_state_, state);
  prev_state_ = curr_state_;
  curr_state_ = state;
  switch (state)
  {
    case IDLE_STATE:
      StateTransition(new IdleState(this));
      break;
    case DIAGNOSTICS_STATE:
      StateTransition(new DiagnosticsState(this));
      break;
    case DANGERZONE_STATE:
      StateTransition(new DangerZoneState(this));
      break;
    case EXP_SAFETYZONE_STATE:
      StateTransition(new ExpSafetyZoneState(this));
      break;
    case LOW_VOLTAGE_STATE:
      StateTransition(new LowVoltageState(this));
      break;
    case ESTOP_STATE:
      StateTransition(new EStopState(this));
      break;
    default:
      break;
  }
}

/**
 * @name        SetDiagnosticsStatus()
 * @brief       Sets the diagnostics monitor status
 * @param[in]   status: enum to set diagstostics status to
 */
void SMManager::SetDiagnosticsStatus(ErrorStatus status)
{
  sm_manager_diagnostics_status_ = status;
}

/**
 * @name        SetExpSafetyZoneStatus()
 * @brief       Sets the expanding safety zone monitor status
 * @param[in]   status: enum to set exp safetyzone status to
 */
void SMManager::SetExpSafetyZoneStatus(ErrorStatus status)
{
  sm_manager_exp_safetyzone_status_ = status;
}

/**
 * @name        SetDangerZoneStatus()
 * @brief       Sets the danger zone monitor status
 * @param[in]   status: enum to set dangerzone status to
 */
void SMManager::SetDangerZoneStatus(ErrorStatus status)
{
  sm_manager_dangerzone_status_ = status;
}

/**
 * @name        SetEstopStatus()
 * @brief       Sets the Estop status
 * @param[in]   status: enum to set estop status  to
 */
void SMManager::SetEStopStatus(ErrorStatus status)
{
  sm_manager_estop_status_ = status;
}

/**
 * @name        SetLowVoltageStatus()
 * @brief       Sets the low voltage status
 * @param[in]   status: enum to set low voltage status to
 */
void SMManager::SetLowVoltageStatus(ErrorStatus status)
{
  sm_manager_low_voltage_status_ = status;
}

/**
 * @name        SetLocalControllerSafetyStatus()
 * @brief       Sets the local controller safety status
 * @param[in]   status: enum to set the local controller safety status
 */
void SMManager::SetLocalControllerSafetyStatus(ErrorStatus status)
{
  sm_manager_local_controller_safety_status_ = status;
}

/**
 * @name        SetDiagnosticsTimeout
 * @brief       Sets the diagnostics timeout value
 * @param[in]   val: value to set the diagnostics timeout to
 */
void SMManager::SetDiagnosticsTimeout(double val)
{
  sm_manager_diagnostics_timeout_ = val;
}

/**
 * @name        SetObstacleTimeout
 * @brief       Sets the obstacle timeout value
 * @param[in]   val: value to set the obstacle timeout to
 */
void SMManager::SetObstacleTimeout(double val)
{
  sm_manager_obstacle_timeout_ = val;
}

/**
 * @name        SetSkipDangerObstacleReset
 * @brief       Sets the skip_danger_obstacle_reset flag
 * @param[in]   reset: bool to set the danger obstacle reset flag to
 */
void SMManager::SetSkipDangerObstacleReset(bool reset)
{
  sm_manager_skip_danger_obstacle_reset_ = reset;
}

/**
 * @name        SetSmFailureReset
 * @brief       Sets the SmFailure reset flag
 * @param[in]   reset: bool to set the SMFailure reset flag to
 */
void SMManager::SetSmFailureReset(bool reset)
{
  sm_manager_failure_reset_ = reset;
}

/**
 * @name        SetDangerZoneZeroVelEnabled
 * @brief       Sets the  SetDangerZoneZeroVelEnabled flag
 * @param[in]   status: bool to set SetDangerZoneZeroVelEnabled to
 */
void SMManager::SetDangerZoneZeroVelEnabled(bool status)
{
  sm_manager_enable_dangerzone_zero_vel_ = status;
}

/**
 * @name        SetEnableSafetyMonitor
 * @brief       Sets the enable safety monitor flag
 * @param[in]   status: bool to set enable safety monitor flag to
 */
void SMManager::SetEnableSafetyMonitor(bool status)
{
  sm_manager_enable_safety_monitor_ = status;
}

/**
 * @name        SetEnableLowVoltage
 * @brief       Sets the enable low voltage flag
 * @param[in]   status: bool to set enable low voltage flag to
 */
void SMManager::SetEnableLowVoltage(bool status)
{
  sm_manager_enable_low_voltage_monitor_ = status;
}

/**
 * @name        SetEnableDiagnostics
 * @brief       Sets the enable diagnostics flag
 * @param[in]   status: bool to set enable diagnostics flag to
 */
void SMManager::SetEnableDiagnostics(bool status)
{
  sm_manager_enable_diagnostics_monitor_ = status;
}

/**
 * @name        SetEnableDangerZone
 * @brief       Sets the enable danger zone flag
 * @param[in]   status: bool to set enable danger zone flag to
 */
void SMManager::SetEnableDangerZone(bool status)
{
  sm_manager_enable_danger_zone_ = status;
}

/**
 * @name        SetEnableExpSafetyZone
 * @brief       Sets the enable exp safety zone flag
 * @param[in]   status: bool to set enable exp safety zone flag to
 */
void SMManager::SetEnableExpSafetyZone(bool status)
{
  sm_manager_enable_exp_safety_zone_ = status;
}

/**
 * @name        SetDiagnosticsDescription()
 * @brief       Sets the diagnostics description
 * @param[in]   description: string to set the diagnostics description to
 */
void SMManager::SetDiagnosticsDescription(std::string description)
{
  sm_manager_diagnostics_description_ = description;
}

/**
 * @name        SetUIState()
 * @brief       Sets UI State
 * @param[in]   description: UI State value
 */
void SMManager::SetUIState(int val)
{
  ui_state_ = val;
}

/**
 * @name        RestartFailureStopWatch
 * @brief       Restart the failure stop watch
 */
void SMManager::RestartFailureStopWatch()
{
  sm_manager_failure_stopwatch_.restart();
}

//////////////////////
/* Getter Functions */
//////////////////////

/**
 * @name        GetState()
 * @brief       Gets the current state 
 */
States SMManager::GetState() const
{
  return curr_state_;
}

/**
 * @name        GetEStopStatus()
 * @brief       Gets the estop_status flag 
 */
ErrorStatus SMManager::GetEStopStatus() const
{
  return sm_manager_estop_status_;
}

/**
 * @name        GetSmFailureReset
 * @brief       Gets the SmFailure reset flag
 */
bool SMManager::GetSmFailureReset() const
{
  return sm_manager_failure_reset_;
}

/**
 * @name        GetDiagnosticsStatus()
 * @brief       Gets the diagnostics_status flag 
 */
ErrorStatus SMManager::GetDiagnosticsStatus() const
{
  return sm_manager_diagnostics_status_;
}

/**
 * @name        GetLowVoltageStatus()
 * @brief       Gets the low_voltage_status flag 
 */
ErrorStatus SMManager::GetLowVoltageStatus() const
{
  return sm_manager_low_voltage_status_;
}

/**
 * @name        GetDangerZoneStatus()
 * @brief       Gets the danger_zone status flag
 */
ErrorStatus SMManager::GetDangerZoneStatus() const
{
  return sm_manager_dangerzone_status_;
}

/**
 * @name        GetExpSafetyZoneStatus()
 * @brief       Gets the exp_safeytyzone status flag
 */
ErrorStatus SMManager::GetExpSafetyZoneStatus() const
{
  return sm_manager_exp_safetyzone_status_;
}

/**
 * @name        GetLocalControllerSafetyStatus()
 * @brief       Gets the local controller safety status flag
 */
ErrorStatus SMManager::GetLocalControllerSafetyStatus() const
{
  return sm_manager_local_controller_safety_status_;
}

/**
 * @name        GetDangerZoneZeroVelEnabled()
 * @brief       Gets the dangerzone_zero_vel_enabled flag 
 */
bool SMManager::GetDangerZoneZeroVelEnabled() const
{
  return sm_manager_enable_dangerzone_zero_vel_;
}

/**
 * @name        GetDiagnosticsTimeout()
 * @brief       Gets the diagnostics_timeout flag 
 */
double SMManager::GetDiagnosticsTimeout() const
{
  return sm_manager_diagnostics_timeout_;
}

/**
 * @name        GetSkipDangerObstacleReset
 * @brief       Gets the skip_danger_obstacle_reset flag
 */
bool SMManager::GetSkipDangerObstacleReset() const
{
  return sm_manager_skip_danger_obstacle_reset_;
}

/**
 * @name        getfailurestopwatchelapsed()
 * @brief       gets the sm_manager_failure_stopwatch_ elapsed time 
 */
ecl::TimeStamp SMManager::GetFailureStopWatchElapsed()
{
  return sm_manager_failure_stopwatch_.elapsed();
}

/**
 * @name        GetObstacleTimeout()
 * @brief       Gets the obstacle_timeout value
 */
double SMManager::GetObstacleTimeout() const
{
  return sm_manager_obstacle_timeout_;
}


/**
 * @name        GetEnableSafetyMonitor()
 * @brief       Gets the enable safety monitor flag
 */
bool SMManager::GetEnableSafetyMonitor() const
{
  return sm_manager_enable_safety_monitor_;
}

/**
 * @name        GetEnableLowVoltage()
 * @brief       Gets the enable low voltage flag
 */
bool SMManager::GetEnableLowVoltage() const
{
  return sm_manager_enable_low_voltage_monitor_;
}

/**
 * @name        GetEnableDiagnostics()
 * @brief       Gets the enable diagnostics flag
 */
bool SMManager::GetEnableDiagnostics() const
{
  return sm_manager_enable_diagnostics_monitor_;
}

/**
 * @name        GetEnableDangerZone()
 * @brief       Gets the enable danger zone flag
 */
bool SMManager::GetEnableDangerZone() const
{
  return sm_manager_enable_danger_zone_;
}

/**
 * @name        GetEnableExpSafetyZone()
 * @brief       Gets the enable exp safety zone flag
 */
bool SMManager::GetEnableExpSafetyZone() const
{
  return sm_manager_enable_exp_safety_zone_;
}

/**
 * @name        GetDiagnosticsDescription()
 * @brief       Gets the diagnostics description 
 */
std::string SMManager::GetDiagnosticsDescription() const
{
  return sm_manager_diagnostics_description_;
}

/**
 * @name        GetUIState()
 * @brief       Gets UI State 
 */
int SMManager::GetUIState() const
{
  return ui_state_;
}

/**
 * @name        PublishState
 * @brief       function that publishes the current state of the safety monitor
 * @param[in]   state_msg_: publish state msg
 */
void SMManager::PublishState(std_msgs::String state_msg_)
{
  ros_->PublishStateMsg(state_msg_);
}

/**
 * @name        PublishSMStatus
 * @brief       function the sm status for GUI update
 * @param[in]   sm_status: error message to publish
 */
void SMManager::PublishSMStatus(avidbots_msgs::sm_status sm_status)
{
  ros_->PublishSMStatus(sm_status);
}

/**
 * @name    PublishZeroVelocity()
 * @brief   Publishes zero velocity to mobile/base and vel_cmd_muxer
 */
void SMManager::PublishZeroVelocity()
{
  ros_->PublishZeroVelocity();
}

/**
 * @name    PublishMCUOutputCommand()
 * @brief   Sets MCU to Safety zone EState
 */
void SMManager::PublishMCUOutputCommand()
{
  ros_->PublishMCUOutputCommand();
}

/**
 * @name      PublishDangerZoneObstacle
 * @brief     Publishes the danger zone obstacle status
 * @param[in] status: the status to publish
 */
void SMManager::PublishDangerZoneObstacle(bool status)
{
  ros_->PublishDangerZoneObstacle(status);
}
