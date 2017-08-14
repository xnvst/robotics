/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	states_manager.cpp
 * @brief	Source File containing the StatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/states_manager.h"

/**
 * @name 	StatesManager
 * @brief	Default constructor
 */
StatesManager::StatesManager()
{
}

/**
 * @name  ~StatesManager
 * @brief Destructor
 */
StatesManager::~StatesManager() {}

/**
 * @name  ~UpgradeState
 * @brief Destructor
 */
void StatesManager::UpgradeState(uint8_t value, uint8_t &status)
{
  if (status < value)
  {
    status = value;
  }
  return;
}
