/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name   roboteq_steer_system_motor.h
 * @brief  Motor controller class for RoboteQ steer system motor drivers
 * @author Yoohee Choi
 */

#ifndef AVIDBOTS_MC_DRIVER_ROBOTEQ_STEER_SYSTEM_MOTOR_H
#define AVIDBOTS_MC_DRIVER_ROBOTEQ_STEER_SYSTEM_MOTOR_H

// ROS
#include <ros/ros.h>

// Local
#include "avidbots_library/socket_can/socket_can_open.h"
#include "avidbots_mc_driver/roboteq_drive_system_motor.h"

class RoboteqSteerSystemMotor : public RoboteqDriveSystemMotor
{
private:
  bool read_calibration_status_;

public:
  virtual int Init(SocketCANOpenProtocol& can_socket, bool read_voltage);

  virtual int RefreshReadings(int sequence);

  // Calibration status check read back from the controller
  bool calibration_;

  // Queues for listening to CAN commands
  boost::lockfree::spsc_queue<struct can_frame,
                              boost::lockfree::capacity<10>> calibration_queue_;
};

#endif  // AVIDBOTS_MC_DRIVER_ROBOTEQ_STEER_SYSTEM_MOTOR_H

