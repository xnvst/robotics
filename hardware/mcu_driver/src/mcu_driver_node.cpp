/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	mcu_driver_node.cpp
 * @brief	Source File containing the Avidbots MCU node class
 * @author	Pablo Molina, Jake Park
 */

#include "avidbots_mcu_driver/mcu_driver.h"
#include "avidbots_mcu_driver/mcu_driver_factory.h"
#include "avidbots_library/get_param/get_param_util.h"

#include <string>

// ------
//  MAIN
// ------

int main(int argc, char** argv )
{
  // Creating ROS variables
  ros::init(argc, argv, "mc_node",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // Creating a mcu driver based on the current robot's configuration
  std::string motion_model;
  GetParamUtil::GetParam("/robot_properties/motion_model", motion_model, "bicycle");

  AvidbotsMCUDriverFactory mcu_driver_factory;
  AvidbotsMCUDriver *mcu_driver = mcu_driver_factory.createMCDriver(motion_model);

  // Connect to device and init device
  int status;
  if (mcu_driver)
  {
    status  = SocketCANOpen::kCANOpenSuccess;
    status      = mcu_driver->InitDriver(private_nh, &node);

    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      ROS_INFO_STREAM("*** Starting publish controller status loop ***");
      // The start publish loop is a locking call
      mcu_driver->StartPublishStatusLoop();
    }
  }
  else
  {
    status = -1;
  }

  delete mcu_driver;
  return status;
}
