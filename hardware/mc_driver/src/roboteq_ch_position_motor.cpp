/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name     roboteq_ch_position_motor.cpp
 * @brief    Implementation of RoboteqCHPositionMotor
 * @author   Tim Wu, Jake Park
 */

// CPP
#include <stdint.h>

// Local
#include "avidbots_mc_driver/motor_driver.h"
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_mc_driver/roboteq_ch_position_motor.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can_open_constants.h"
#include "avidbots_library/socket_can/socket_can_open_constants.h"

/**
 * @name       Init
 * @brief      initialize the CAN device
 * @param[in]  can_socket: the CAN socket binded
 * @param[in]  read_voltage: whether voltage is read
 * @return     whether initialization has succeeded
 */
int RoboteqCHPositionMotor::Init(SocketCANOpenProtocol& can_socket, bool read_voltage)
{
  can_socket_ = &can_socket;

  /* CH position has only one value to update, but it doesn't need to be updated
   * at main loop's rate. Desired rate was ~ 13Hz which is every 3rd times in
   * the main loop. Hence, setting the non_critical_to_update_ to 3 and it would
   * be part of different round robin sequence than other motor drivers. */
  non_critical_to_update_ = 3;
  read_voltage_ = read_voltage;

  // Listener set up for heartbeat
  struct CANOpenListenMask listen_mask =
      { RoboteqNxtGenCANOpen::kHeartBeat+can_id_, 0, {} };
  can_socket_->AddFrameListener(listen_mask, heartbeat_queue_);
  last_heartbeat_read_ = can_socket_->GetCurrentTimeStamp();

  // *** HACK: By necessity, this motor object will be used to read the left rear motor ticks ***
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage4Byte, RoboteqNxtGenCANOpen::kMCReadEncTickB2,
      RoboteqNxtGenCANOpen::kMCReadEncTickB1, RoboteqNxtGenCANOpen::kMCReadEncTickSubIndex1
    }
  };
  can_socket_->AddFrameListener(listen_mask, ticks_queue_);

  int ch_pos_sub_id = 1;
  GetParamUtil::GetParam("/mc_properties/ch_pos_sub_id", ch_pos_sub_id, 1);
  user_int_var_id_ = ((ch_pos_sub_id == 1)?
                        RoboteqNxtGenCANOpen::kMCSetUserIntVar1SubIndex :
                        RoboteqNxtGenCANOpen::kMCSetUserIntVar2SubIndex);

  ROS_DEBUG_STREAM("RoboteqCHPositionMotor::Init on can id " << can_id_ <<
                   ", ch_pos_sub_id " << ch_pos_sub_id);
  return 0;
}

/**
 * @name    SetRPM
 * @brief   set RPM for squeegee position motor
 * @return  whether setting RPM has succeeded (0)
 */
int RoboteqCHPositionMotor::SetRPM()
{
  if (error_state_)
  {
    return MotorDriver::kErrorStateThrown;
  }
  uint32_t rpm = RPM_cmd_.load();
  can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage4Byte,
                       RoboteqNxtGenCANOpen::kMCSetUserIntVarB1,
                       RoboteqNxtGenCANOpen::kMCSetUserIntVarB2,
                       user_int_var_id_,
                       rpm);
  return 0;
}

/**
 * @name       RefreshReadings
 * @brief      read each value and return an error code if something went wrong
 * @param[in]  sequence: specific measurement ID
 * @return     SocketCANOpen::kCANOpenSuccess, or error code from SocketCANOpen
 */
int RoboteqCHPositionMotor::RefreshReadings(int sequence)
{
  ROS_DEBUG("[CHPositionmotor] RefreshReadings sequence: %d", sequence);

  // Get current time
  uint64_t now = can_socket_->GetCurrentTimeStamp();

  // Read heart beat
  uint8_t heartbeat;
  int status = can_socket_->ReadHeartbeatFromQueue(heartbeat, heartbeat_queue_, 0);

  // Set error state based on status
  if (status == SocketCANOpen::kCANOpenSuccess)
  {
    error_state_ = false;
    last_heartbeat_read_ = now;
    ROS_DEBUG_STREAM("Read heartbeat on motor #" << can_id_ << ": " << heartbeat);
  }
  // Check the heartbeat timeout
  if (now - last_heartbeat_read_ > MotorDriver::kCANHeartbeatTimeoutMS)
  {
    error_state_ = true;
    ROS_WARN_STREAM("Heartbeat CH.MC. timeout on #" << can_id_);
    return MotorDriver::kErrorHeartbeatTimeout;
  }

  // *** HACK: Send request for the left rear motor ticks ***
  can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage4Byte, RoboteqNxtGenCANOpen::kMCReadEncTickB1,
                        RoboteqNxtGenCANOpen::kMCReadEncTickB2, RoboteqNxtGenCANOpen::kMCReadEncTickSubIndex1);

  if (sequence == 0)
  {
    SetRPM();
  }

  // *** HACK: Read the left rear motor ticks ***
  uint32_t ticks = 0;
  status = can_socket_->ReadSDOFromQueue(ticks,
                                         ticks_queue_,
                                         RoboteqCHPositionMotor::kCANReadTimeoutMS);
  if (status == SocketCANOpen::kCANOpenSuccess)
  {
    ticks_ = ticks;
  }
  else
  {
    error_state_ = true;
    ROS_WARN_STREAM("Ticks CH.MC. reading timeout on CANID: " << can_id_
                    << " sequence: " << sequence << " can_status: " << status);

    return RoboteqCHPositionMotor::kErrorReadValueTimeout;
  }

  if (error_state_)
  {
    return MotorDriver::kErrorStateThrown;
  }

  return MotorDriver::kSuccess;
}

/**
 * @name       SetManualOverride
 * @brief      do nothing
 * @param[in]  manual override (true) / normal operation (false)
 */
void RoboteqCHPositionMotor::SetManualOverride(bool manual_override) {}
