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

#include "avidbots_mc_driver/avidbots_mc_driver_factory.h"

// C++
#include <string>

#include "avidbots_mc_driver/bicycle_mc_driver.h"
#include "avidbots_mc_driver/kiwi_mc_driver.h"
#include "avidbots_msgs/constants.h"

/**
 * @name  createMCDriver
 * @brief creates an instance of AvidbotsMCCANOpenDriver based the current robot running this code
 * @param[in] type: description for the type of drive system being used currently
 * @return[out]: instance of AvidbotsMCCANOpenDriver if type is defined correctly; otherwise return null pointer
 */
AvidbotsMCCANOpenDriver *AvidbotsMCDriverFactory::createMCDriver(const std::string &type)
{
  if (type == KIWI)
  {
    return new KiwiMCDriver();
  }
  else if (type == BICYCLE)
  {
    return new BicycleMCDriver();
  }
  else
  {
    return NULL;
  }
}
