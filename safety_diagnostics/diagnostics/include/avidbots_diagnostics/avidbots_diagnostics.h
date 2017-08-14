/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_diagnostics.h
 * @brief	Header file containing the Avidbots Diagnostics class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_H
#define AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_H

// CPP
#include <string>

// BOOST
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/functional/hash.hpp>

// LCOAL
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/diagnostics_states.h>
#include <avidbots_msgs/diagnostics.h>
#include <avidbots_msgs/sensor_toggle.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"
#include "avidbots_diagnostics/mcu_states_manager.h"
#include "avidbots_diagnostics/mc_states_manager.h"
#include "avidbots_diagnostics/urg_states_manager.h"
#include "avidbots_diagnostics/camera_states_manager.h"
#include "avidbots_diagnostics/sensor_safety_states_manager.h"
#include "avidbots_diagnostics/global_diagnostics_manager.h"

class AvidbotsDiagnostics
{
public:
  static AvidbotsDiagnostics& GetInstance()
  {
      static AvidbotsDiagnostics instance;
      return instance;
  }
  ~AvidbotsDiagnostics();

  void Init(const ros::NodeHandle& private_nh,
                  ros::NodeHandle* node_handle);
  void PublishDiagnosticsGlobalUpdate();

  MCUStatesManager mcu_states_mgr_;
  MCStatesManager mc_states_mgr_;
  URGStatesManager urg_states_mgr_;
  CameraStatesManager camera_left_states_mgr_;
  CameraStatesManager camera_right_states_mgr_;
  SensorSafetyStatesManager sensor_safety_states_mgr_;

  GlobalDiagnosticsManager global_diagnostics_mgr_;

  // ROS publisher/subscriber
  ros::NodeHandle*  node_handle_;

  ros::Subscriber   diagnostics_states_sub_;
  ros::Subscriber   urg_diagnostics_states_sub_;
  ros::Subscriber   camera_left_diagnostics_states_sub_;
  ros::Subscriber   camera_right_diagnostics_states_sub_;
  ros::Subscriber   sensor_toggle_sub_;
  ros::Publisher    diagnostics_global_pub_;

  void GetParamSettings(const ros::NodeHandle& private_nh);
  void ManagePubAndSub();

  // CallBacks
  void DiagnosticsStatesCallback(const avidbots_msgs::diagnostics_statesPtr &diagnostics_states);
  void URGDiagnosticsStatesCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &urg_diagnostics_array);
  void CameraLeftDiagnosticsStatesCallback(const avidbots_msgs::diagnostics_statesPtr &diagnostics_states);
  void CameraRightDiagnosticsStatesCallback(const avidbots_msgs::diagnostics_statesPtr &diagnostics_states);
  void SensorToggleCallback(const avidbots_msgs::sensor_toggle &sensor_toggle);

private:
  AvidbotsDiagnostics();
  bool global_enable_;
  avidbots_msgs::diagnostics diagnostics_msg_;
};

#define AvidbotsDiagnosticsInstance \
  AvidbotsDiagnostics::GetInstance()

#endif  // AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_H
