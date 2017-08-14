/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name     roboteq_ch_position_motor.h
 * @brief    Motor controller class for RoboteQ cleaning head position driver
 * @author   Tim Wu
 */

#ifndef AVIDBOTS_MC_DRIVER_ROBOTEQ_CH_POSITION_MOTOR_H
#define AVIDBOTS_MC_DRIVER_ROBOTEQ_CH_POSITION_MOTOR_H

// C++
#include <stdint.h>

// Local
#include "avidbots_mc_driver/motor_driver.h"
#include "avidbots_library/socket_can/socket_can_open_constants.h"


class RoboteqCHPositionMotor : public MotorDriver
{
public:
  int SetRPM();
  int RefreshReadings(int sequence);
  void SetManualOverride(bool manual_override);
  int Init(SocketCANOpenProtocol& can_socket, bool read_voltage);

private:
  uint8_t user_int_var_id_;
};

#endif  // AVIDBOTS_MC_DRIVER_ROBOTEQ_CH_POSITION_MOTOR_H
