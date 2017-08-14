/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    mcu_driver_factory.cpp
 * @brief	Source File containing the Avidbots MCU driver class
 * @author	Jake Park
 */

#include "avidbots_mcu_driver/mcu_driver_factory.h"

#include "avidbots_msgs/constants.h"
#include "avidbots_mcu_driver/shiny_mcu_driver.h"
#include "avidbots_mcu_driver/wheely_mcu_driver.h"

#include <string>

/**
 * @name  createMCDriver
 * @brief creates an instance of AvidbotsMCCANOpenDriver based the current robot running this code
 * @param[in] type: description for the type of drive system being used currently
 * @return[out]: instance of AvidbotsMCCANOpenDriver if type is defined correctly; otherwise return null pointer
 */
AvidbotsMCUDriver *AvidbotsMCUDriverFactory::createMCDriver(const std::string &type)
{
  if (type == KIWI)
  {
    return new ShinyMCUDriver();
  }
  else if (type == BICYCLE)
  {
    return new WheelyMCUDriver();
  }
  else
  {
    return NULL;
  }
}
