/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name roboteq_drive_system_motor.h
 * @brief	Motor controller class for RoboteQ drive system motor drivers
 * @author	Joseph Duchesne
 */

#ifndef AVIDBOTS_MC_DRIVER_ROBOTEQ_DRIVE_SYSTEM_MOTOR_H
#define AVIDBOTS_MC_DRIVER_ROBOTEQ_DRIVE_SYSTEM_MOTOR_H

// Standard
#include <string>
#include <stdint.h>

// Boost
#include <boost/lockfree/spsc_queue.hpp>

// ROS
#include <ros/ros.h>

// Local
#include "avidbots_library/socket_can/socket_can_open.h"
#include "avidbots_mc_driver/motor_driver.h"


class RoboteqDriveSystemMotor : public MotorDriver
{
private:
public:
  virtual int Init(SocketCANOpenProtocol& can_socket, bool read_voltage);

  virtual int SetRPM();

  virtual int RefreshReadings(int sequence);

  virtual void SetManualOverride(bool manual_override);

  virtual int SetShutdown(bool on);
};

#endif  // AVIDBOTS_MC_DRIVER_ROBOTEQ_DRIVE_SYSTEM_MOTOR_H

