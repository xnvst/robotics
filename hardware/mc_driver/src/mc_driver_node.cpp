/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	Avidbots MC driver
 * @brief	Header File containing the Can open device protocol class
 * @author	Pablo Molina, Jake Park
 */

// CPP
#include <signal.h>
#include <string>

// LOCAL
#include "avidbots_mc_driver/avidbots_mc_driver.h"
#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_mc_driver/avidbots_mc_driver_factory.h"

/** Global variables
 * NOTE:
 * This has to be global to be accessable in the sigint handler
 */

AvidbotsMCCANOpenDriver* mc_driver;

/**
 * @name 	sigint_handler
 * @brief	Interrupt handler - sends shutdown signal to driver
 * @param[in] s: signal itself
 */
void sigint_handler(int s)
{
  ROS_INFO_STREAM("*** MC Driver: Caught kill signal: "
                   << s << ". Exiting... ***");
  mc_driver->ExitDriverThread();
}

// ------
//  MAIN
// ------
int main(int argc, char** argv )
{
  signal(SIGINT, sigint_handler);
  // Creating ROS variables
  ros::init(argc, argv, "mc_node",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle node;

  std::string motion_model;
  GetParamUtil::GetParam("/robot_properties/motion_model", motion_model, "kiwi");

  AvidbotsMCDriverFactory mc_driver_factory;

  mc_driver = mc_driver_factory.createMCDriver(motion_model);

  int status  = SocketCANOpen::kCANOpenSuccess;
  // Connect to device and init device
  if (mc_driver)
  {
    status      = mc_driver->InitDriver(&node);

    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      ROS_INFO_STREAM("*** Starting publish controller status loop ***");
      // The start publish loop is a locking call
      mc_driver->StartPublishStatusTimer();
    }
  }
  else
  {
    status = -1;
  }

  delete mc_driver;
  return status;
}
