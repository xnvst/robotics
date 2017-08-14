/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	Avidbots MC driver
 * @brief	Header File containing the Roboteq Next Gen CAN implementation
 * @author	Pablo Molina
 */

#ifndef AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_H
#define AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_H

#include "avidbots_library/socket_can/socket_can_open.h"
#include <string>
#include <stdint.h>


class RoboteqNxtGenCANOpenController
{
private:
public:
  SocketCANOpenProtocol can_socket_;

  int InitPort(const std::string& can_port, const int& can_timeout);

  int SetMotorSpeed(const uint8_t& can_id, const int& motor_cmd);

  int GetEncoderTicks(const uint8_t& can_id, int& motor_ticks);

  int GetEncoderRPM(const uint8_t& can_id, int16_t& enc_rpm);

  int GetBrushlessTicks(const uint8_t& can_id, int& brushless_ticks);

  int GetMCTemp(const uint8_t& can_id, int8_t& mc_temp);

  int GetMotorAmps(const uint8_t& can_id, int16_t& motor_amps);

  int GetMainBattVoltage(const uint8_t& can_id, uint16_t& batt_volt);


  void ClosePort();
};

#endif  // AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_H
