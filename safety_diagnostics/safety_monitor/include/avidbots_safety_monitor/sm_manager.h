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
 * @brief   Header file for SafetyMonitorManager class
 * @author  Keshav Iyengar, Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SM_MANAGER_H
#define AVIDBOTS_SAFETY_MONITOR_SM_MANAGER_H

#include <ecl/time.hpp>

// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/states/state_base.h"
#include "avidbots_safety_monitor/sm_constants.h"

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

// Other
#include <visualization_msgs/Marker.h>

class StateBase;
class ROSInterface;

/**
 * @name    SafetyMonitorManager
 * @brief   main state machine.
 */
class SMManager
{
  friend class StateBase;
public:
  SMManager();
  ~SMManager();

  void Initialize();
  void Update();
  void StateTransition(StateBase* state);

  /* Events */
  void SMStateEvent(Events event);

  /* Setters */
  void SetDiagnosticsStatus(ErrorStatus status);
  void SetExpSafetyZoneStatus(ErrorStatus status);
  void SetDangerZoneStatus(ErrorStatus status);
  void SetEStopStatus(ErrorStatus status);
  void SetLowVoltageStatus(ErrorStatus status);
  void SetLocalControllerSafetyStatus(ErrorStatus status);
  void SetSmFailureReset(bool reset);
  void SetCount(int count);
  void SetState(States state);
  void SetDiagnosticsTimeout(double val);
  void SetSkipDangerObstacleReset(bool reset);
  void SetObstacleTimeout(double val);
  void SetEnableLowVoltage(bool status);
  void SetEnableDiagnostics(bool status);
  void SetEnableSafetyMonitor(bool status);
  void SetEnableDangerZone(bool status);
  void SetEnableExpSafetyZone(bool status);
  void SetDangerZoneZeroVelEnabled(bool status);
  void SetDiagnosticsDescription(std::string description);
  void SetUIState(int val);

  /* Getters */
  States GetState() const;
  bool GetSmFailureReset() const;
  ErrorStatus GetEStopStatus() const;
  ErrorStatus GetDiagnosticsStatus() const;
  ErrorStatus GetDangerZoneStatus() const;
  ErrorStatus GetExpSafetyZoneStatus() const;
  ErrorStatus GetLowVoltageStatus() const;
  ErrorStatus GetLocalControllerSafetyStatus() const;
  ecl::TimeStamp GetFailureStopWatchElapsed();
  double GetDiagnosticsTimeout() const;
  bool GetSkipDangerObstacleReset() const;
  double GetObstacleTimeout() const;
  bool GetEnableLowVoltage() const;
  bool GetEnableDiagnostics() const;
  bool GetEnableSafetyMonitor() const;
  bool GetEnableDangerZone() const;
  bool GetEnableExpSafetyZone() const;
  bool GetDangerZoneZeroVelEnabled() const;
  std::string GetDiagnosticsDescription() const;
  int GetUIState() const;

  void RestartFailureStopWatch();

  /* Publish MSG */
  void PublishState(std_msgs::String state_msg_);
  void PublishSMStatus(avidbots_msgs::sm_status sm_status);
  void PublishZeroVelocity();
  void PublishMCUOutputCommand();
  void PublishDangerZoneObstacle(bool status);

private:
  /* ROS Interface */
  ROSInterface*         ros_;

  /* State */
  StateBase*            state_;
  States                prev_state_;
  States                curr_state_;
  States                next_state_;

  Events                curr_event_;

  /* UI State */
  int ui_state_;

  /* State Transition Variables, true -- ok, false -- error */

  ErrorStatus sm_manager_diagnostics_status_;
  ErrorStatus sm_manager_exp_safetyzone_status_;
  ErrorStatus sm_manager_dangerzone_status_;
  ErrorStatus sm_manager_low_voltage_status_;
  ErrorStatus sm_manager_estop_status_;
  ErrorStatus sm_manager_local_controller_safety_status_;

  std::string sm_manager_diagnostics_description_;

  /* Failure Reset Variable, true if the safety monitor failure is reset */
  bool sm_manager_failure_reset_;

  /* Parameters */
  double sm_manager_diagnostics_timeout_;
  double sm_manager_obstacle_timeout_;
  bool sm_manager_skip_danger_obstacle_reset_;

  /* Enable Flags */
  bool sm_manager_enable_low_voltage_monitor_;
  bool sm_manager_enable_diagnostics_monitor_;
  bool sm_manager_enable_safety_monitor_;
  bool sm_manager_enable_danger_zone_;
  bool sm_manager_enable_exp_safety_zone_;
  bool sm_manager_enable_dangerzone_zero_vel_;

  ecl::StopWatch sm_manager_failure_stopwatch_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_SM_MANAGER_H
