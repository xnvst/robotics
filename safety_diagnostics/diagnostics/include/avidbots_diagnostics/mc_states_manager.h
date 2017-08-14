/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	mc_states_manager.h
 * @brief	Header file containing the MCStatesManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_DIAGNOSTICS_MC_STATES_MANAGER_H
#define AVIDBOTS_DIAGNOSTICS_MC_STATES_MANAGER_H

#include <vector>

// LCOAL
#include "avidbots_diagnostics/states_manager.h"

class MCStatesManager : public StatesManager
{
public:
  MCStatesManager();
  virtual ~MCStatesManager();
  virtual void Init(uint8_t hardware_id);
  virtual void UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                 avidbots_msgs::diagnostics &diagnostics_msg);

private:
  int8_t states_;
};

#endif  // AVIDBOTS_DIAGNOSTICS_MC_STATES_MANAGER_H