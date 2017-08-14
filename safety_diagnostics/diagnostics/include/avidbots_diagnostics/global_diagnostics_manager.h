/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	global_diagnostics_manager.h
 * @brief	Header file containing the GlobalDiagnosticsManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_DIAGNOSTICS_GLOBAL_DIAGNOSTICS_MANAGER_H
#define AVIDBOTS_DIAGNOSTICS_GLOBAL_DIAGNOSTICS_MANAGER_H

// CPP
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>

// BOOST
#include <boost/lockfree/spsc_queue.hpp>

// LCOAL
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/diagnostics_states.h>
#include <avidbots_msgs/diagnostics.h>
#include "avidbots_diagnostics/avidbots_diagnostics_constants.h"

class GlobalDiagnosticsManager
{
public:
  GlobalDiagnosticsManager();
  virtual ~GlobalDiagnosticsManager();
  bool AddQueue(const avidbots_msgs::diagnostics_states &states);
  bool ReadQueue(avidbots_msgs::diagnostics_states &states);

private:
  boost::lockfree::spsc_queue<avidbots_msgs::diagnostics_states,
                              boost::lockfree::capacity<20>>
                              diagnostics_states_queue_;
};

#endif  // AVIDBOTS_DIAGNOSTICS_GLOBAL_DIAGNOSTICS_MANAGER_H
