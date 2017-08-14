/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  mcu_driver_factory.h
 * @brief Header File containing the AvidbotsMCUDriverFactory class
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MCU_DRIVER_MCU_DRIVER_FACTORY_H
#define AVIDBOTS_MCU_DRIVER_MCU_DRIVER_FACTORY_H

#include "avidbots_mcu_driver/mcu_driver.h"

// C++
#include <string>

class AvidbotsMCUDriverFactory
{
public:
  AvidbotsMCUDriver *createMCDriver(const std::string &type);
};

#endif  // AVIDBOTS_MCU_DRIVER_MCU_DRIVER_FACTORY_H
