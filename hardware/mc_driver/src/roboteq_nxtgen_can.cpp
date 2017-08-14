/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 * @name	Roboteq Next gen
 * @brief	Source file for roboteq next gen class implementation
 * @author	Pablo Molina
 */
#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can_open_constants.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can.h"
#include <stdint.h>
#include <vector>
#include <string>

/**
 * @name 	InitDevice
 * @brief	This function initializes the CAN device
 * @param[in] string can_port: The CAN port to connect to
 */
int RoboteqNxtGenCANOpenController:: InitPort(const std::string& can_port,
                                              const int& can_timeout)
{
  return can_socket_.InitSocket(can_port, can_timeout);
}

/**
 * @name 	SetMotorSpeed
 * @brief	This function sets the speed of a certain node in the network
 * @param[in] uint8_t   can_id: The CAN ID of the motor controller
 * @param[out] uint8_t  motor_cmd: The command of the motor speed
 * @return The result of command
 */
int RoboteqNxtGenCANOpenController::SetMotorSpeed(const uint8_t& can_id,
                                                  const int& motor_cmd)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCSetMotorSpeedB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCSetMotorSpeedB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message
  std::vector<uint8_t> data;
  data.push_back(motor_cmd & 0xFF);
  data.push_back(motor_cmd >> 8 & 0xFF);
  data.push_back(motor_cmd >> 16 & 0xFF);
  data.push_back(motor_cmd >> 24 & 0xFF);


  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage4Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  return return_value;
}

/**
 * @name 	GetMCTemp
 * @brief	This function returns the temperature of the motor controler
 * @param[in] uint8_t   can_id: The CAN ID of the motor controller
 * @param[out] uint8_t  mc_temp: The temperature of the motor controller
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetMCTemp(const uint8_t& can_id,
                                              int8_t& mc_temp)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCTempB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCTempB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message is 0
  std::vector<uint8_t> data;

  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);


  // Get only the first temperature
  mc_temp = data[0];

  return return_value;
}

/**
 * @name    GetMotorAmps
 * @brief   Gets the current in a certain can ID
 * @param[in] can_id: The CAN ID of the controller
 * @param[in/out] motor_amps: current of the motor
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetMotorAmps(const uint8_t& can_id,
                                                 int16_t& motor_amps)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCReadAmpB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCReadAmpB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message is 0
  std::vector<uint8_t> data;

  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  // The incoming message only contains 2 good bytes -
  // Create the motor current from the first two bytes
  motor_amps = data[1] << 8 | data[0];

  return return_value;
}

/**
 * @name    GetMotorTicks
 * @brief   Gets the current ticks in a certain controller
 * @param[in] can_id: The CAN ID of the controller
 * @param[in/out] motor_ticks: The ticks of the motor
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetEncoderTicks(const uint8_t& can_id,
                                                  int& motor_ticks)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCReadEncTickB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCReadEncTickB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message is 0
  std::vector<uint8_t> data;
  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  // This message contains 4 valid bytes
  // Create the motor ticks from the 4 bytes of the message
  motor_ticks = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];

  return return_value;
}


/**
 * @name    GetMotorTicks
 * @brief   Gets the current brushless hall effect ticks in a controller
 * @param[in] can_id: The CAN ID of the controller
 * @param[in/out] motor_ticks: The ticks of the brushless
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetBrushlessTicks(const uint8_t& can_id,
                                                  int& brushless_ticks)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCReadBrlessTickB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCReadBrlessTickB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message is 0
  std::vector<uint8_t> data;
  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  // This message contains 4 valid bytes
  // Create the motor ticks from the 4 bytes of the message
  brushless_ticks = data[2] << 24 | data[2] << 16 | data[1] << 8 | data[0];

  return return_value;
}

/**
 * @name    GetEncoderRPM
 * @brief   Gets the current RPM as reported by encoder
 * @param[in] can_id: The CAN ID of the controller
 * @param[out] enc_rpm: The encoder RPM
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetEncoderRPM(const uint8_t& can_id,
                                                  int16_t& enc_rpm)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCReadEncRPMB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCReadEncRPMB2;
  uint8_t od_sub_index  = 0x01;
  // Data of this message is 0
  std::vector<uint8_t> data;
  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  // This message contains 2 valid bytes
  // Create the motor ticks from the 4 bytes of the message
  enc_rpm = data[1] << 8 | data[0];

  return return_value;
}

/**
 * @name    GetMainBattVoltage
 * @brief   Gets the current voltage of the battery
 * @param[in] can_id: The CAN ID of the controller
 * @param[out] batt_volt: The battery voltage
 * @return The status of the current command sent
 */
int RoboteqNxtGenCANOpenController::GetMainBattVoltage(const uint8_t& can_id,
                                                  uint16_t& batt_volt)
{
  // Object dictionary for this message
  uint8_t od_index_b1   = RoboteqNxtGenCANOpen::kMCReadBattVoltsB1;
  uint8_t od_index_b2   = RoboteqNxtGenCANOpen::kMCReadBattVoltsB2;
  uint8_t od_sub_index  = 0x02;  // this is the subindex for the batt voltage
  // Data of this message is 0
  std::vector<uint8_t> data;
  data.push_back(0x00);

  int return_value = can_socket_.SendSDO(can_id,
                                         SocketCANOpen::kSDOWriteMessage1Byte,
                                         od_index_b1, od_index_b2,
                                         od_sub_index, data);
  // This message contains 2 valid bytes
  // Create the main battery voltage from the 2 bytes of the message
  batt_volt = data[1] << 8 | data[0];

  return return_value;
}


/**
 * @name    ClosePort
 * @brief   Closes the CAN port
 */
void RoboteqNxtGenCANOpenController::ClosePort()
{
  can_socket_.CloseSocket();
}

