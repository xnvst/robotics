/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name  motor_driver.h
 * @brief Virtual motor controller class motor drivers
 * @author  Joseph Duchesne
 */

#ifndef AVIDBOTS_MC_DRIVER_MOTOR_DRIVER_H
#define AVIDBOTS_MC_DRIVER_MOTOR_DRIVER_H

// Standard
#include <string>
#include <stdint.h>

// Boost
#include <boost/lockfree/spsc_queue.hpp>

// ROS
#include <ros/ros.h>

// Local
#include "avidbots_library/socket_can/socket_can_open.h"



class MotorDriver
{
protected:
  SocketCANOpenProtocol *can_socket_;
  const int kCANReadTimeoutMS = 100;  // 100ms
  const int kCANHeartbeatTimeoutMS = 2500;  // 2.5sec
  uint64_t last_heartbeat_read_ = 0;
  std::atomic<uint64_t> last_manual_override_;

  const int kManualOverrideTimeoutMS = 500;  // milliseconds

  bool read_voltage_;

public:
  // constants
  const int kSuccess               =  0;
  const int kErrorHeartbeatTimeout = -1;
  const int kErrorReadValueTimeout = -2;
  const int kErrorStateThrown      = -3;

  // error state
  std::atomic<bool>         error_state_;

  // The RPM set by command
  std::atomic<int>          RPM_cmd_;

  // Manual override
  std::atomic<bool>         manual_override_;

  // CAN ID number (this is int to keep ROS param server happy)
  int                       can_id_;

  // The current in amps meaured on this channel
  std::atomic<int>          amps_;
  // The encoder ticks of this channel
  std::atomic<int>          ticks_;
  // the RPM read back from the controller
  std::atomic<int>          RPM_;
  // the Voltage read back from the controller
  std::atomic<int>          voltage_;

  // Number of sequenced updates that this driver has
  int non_critical_to_update_;

  // queues for listening to CAN commands
  Can_Frame_Cap_10_Queue heartbeat_queue_;

  Can_Frame_Cap_10_Queue amps_queue_;
  Can_Frame_Cap_10_Queue ticks_queue_;
  Can_Frame_Cap_10_Queue RPM_queue_;
  Can_Frame_Cap_10_Queue voltage_queue_;

  virtual int Init(SocketCANOpenProtocol& can_socket, bool read_voltage) = 0;

  virtual int SetRPM() = 0;

  virtual int RefreshReadings(int sequence) = 0;

  virtual void SetManualOverride(bool manual_override) = 0;
};

#endif  // AVIDBOTS_MC_DRIVER_MOTOR_DRIVER_H
