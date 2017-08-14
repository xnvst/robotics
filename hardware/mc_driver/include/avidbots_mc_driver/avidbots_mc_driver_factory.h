/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  avidbots_mc_driver.h
 * @brief Header File containing the AvidbotsMCDriverFactory class
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_FACTORY_H
#define AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_FACTORY_H

#include "avidbots_mc_driver/avidbots_mc_driver.h"

// C++
#include <string>

class AvidbotsMCDriverFactory
{
public:
  AvidbotsMCCANOpenDriver *createMCDriver(const std::string &type);
};

#endif  // AVIDBOTS_MC_DRIVER_AVIDBOTS_MC_DRIVER_FACTORY_H
