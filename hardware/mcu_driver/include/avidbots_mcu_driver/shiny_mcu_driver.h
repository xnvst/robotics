/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  shiny_mcu_driver.h
 * @brief Header File containing the ShinyMCUDriver class
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MCU_DRIVER_SHINY_MCU_DRIVER_H
#define AVIDBOTS_MCU_DRIVER_SHINY_MCU_DRIVER_H

#include "avidbots_mcu_driver/mcu_driver.h"

class ShinyMCUDriver : public AvidbotsMCUDriver
{
protected:
  void GetParamSettings(const ros::NodeHandle& private_nh);

  int InitDriver(const ros::NodeHandle &private_nh, ros::NodeHandle *node_handle);

public:
  void StartPublishStatusLoop();
};

#endif  // AVIDBOTS_MCU_DRIVER_SHINY_MCU_DRIVER_H
