/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	avidbots_can_open.cpp
 * @brief	Header file containing the Avidbots CAN open implementation
 * @author	Pablo Molina
 */
#ifndef AVIDBOTS_MCU_DRIVER_AVIDBOTS_CAN_OPEN_H
#define AVIDBOTS_MCU_DRIVER_AVIDBOTS_CAN_OPEN_H

#include <string>
#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>

#include <avidbots_library/socket_can/socket_can_open.h>
#include <avidbots_library/socket_can/socket_can_open_constants.h>
#include <avidbots_msgs/mcu_output_cmd.h>

class AvidbotsCANOpenController
{
private:
  uint16_t can_id_;
  SocketCANOpenProtocol can_socket_;

  const int kCANReadTimeoutMS = 100;
  const int kCANWaitBetweenReadsUS = 1000;
  const int kCANWaitBetweenWriteUS = 1000;

  Can_Frame_Cap_10_Queue status_variable_queue_;
  Can_Frame_Cap_1_Queue manualmotion_variable_queue_;
  Can_Frame_Cap_10_Queue mcu_version_queue_;

  // Pertaining to writing out data
  std::atomic<bool>    unconfirmed_writes_[avidbots_msgs::mcu_output_cmd::kOutputCount];
  std::atomic<uint8_t> unconfirmed_write_data_[avidbots_msgs::mcu_output_cmd::kOutputCount];

  int GetOutputCommandForCANRequest(struct can_frame);
  int WriteOutputCommand(const uint8_t&, const uint8_t&);

protected:
  int SetVacuumState(uint8_t vacuum_state);
  int SetMainBrushState(uint8_t brush_state);
  int SetSideBrushState(uint8_t brush_state);
  int SetCleaningHeadPosition(uint8_t ch_pos);
  int SetDrivingMode(uint8_t driving_mde);
  int SetPumpSpeed(uint8_t pump_speed);
  int SetBrake(uint8_t value);
  int SetCancelEStop(uint8_t value);
  int SetLEDStatus(uint8_t led_status);
  int SetLEDDirection(uint8_t led_dir);
  int SetSolenoid(uint8_t value);
  int SetSafetyMonitorStatus(uint8_t value);

public:
  // constants
  const int kSuccess               =  0;
  const int kErrorReadValueTimeout = -1;

  AvidbotsCANOpenController();

  int WritePendingMessages();

  int GetStatusVariable(uint8_t& state_variable,
                        uint8_t& estop_status);
  int GetStatusVariable(uint8_t &clean_empty_state,
                        uint8_t &dirty_full_state,
                        uint8_t &estop_status,
                        uint8_t &state_variable);
  int GetManualMotionVariable(uint16_t& steering_val,
                              uint16_t& driving_val);
  int GetMCUFirmwareVersion(uint8_t& major,
                            uint8_t& minor,
                            uint8_t& patch);

  int ProcessOutputCommand(const uint8_t& output_num,
                           const uint8_t& output_val);

  int Init(const std::string& portName,
           const int &timeout_read_ms,
           const uint16_t &can_id);

  void CloseSocket();
};

#endif  // AVIDBOTS_MCU_DRIVER_AVIDBOTS_CAN_OPEN_H
