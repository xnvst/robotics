/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name  roboteq_cleaning_head_motor.cpp
 * @brief Soruce File containing the Roboteq cleaning head motor controller class implementation
 * @author  Joseph Duchesne
 */

#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_mc_driver/roboteq_cleaning_head_motor.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can_open_constants.h"


/**
 * @name  InitDevice
 * @brief This function initializes the CAN device
 * @param[in] string can_port: The CAN port to connect to
 */
int RoboteqCleaningHeadMotor::Init(SocketCANOpenProtocol& can_socket, bool read_voltage)
{
  can_socket_ = &can_socket;

  // assert if can_id_ hasn't been set

  ROS_DEBUG_STREAM("RoboteqBrushedMotor::Init on can id " << can_id_);

  // set up listeners for thigns we're interested in

  // heartbeat
  struct CANOpenListenMask listen_mask = {RoboteqNxtGenCANOpen::kHeartBeat+can_id_, 0, {}};
  can_socket_->AddFrameListener(listen_mask, heartbeat_queue_);
  // don't timeout instantly, give it one interval
  last_heartbeat_read_ = can_socket_->GetCurrentTimeStamp();

  non_critical_to_update_ = 2;

  // Amps
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage2Byte,  RoboteqNxtGenCANOpen::kMCReadAmpB2,
      RoboteqNxtGenCANOpen::kMCReadAmpB1, RoboteqNxtGenCANOpen::kMCReadAmpSubIndex
    }
  };
  can_socket_->AddFrameListener(listen_mask, amps_queue_);

  read_voltage_ = false;

  return 0;
}

/**
 * @name  SetRPM
 * @brief Sets the RPM for the cleaning head motor
 */
int RoboteqCleaningHeadMotor::SetRPM()
{
  if (error_state_)
  {
    return RoboteqCleaningHeadMotor::kErrorStateThrown;
  }

  uint32_t rpm = RPM_cmd_.load();  // copy the atomic value into a uint32_t for sending as 4 byte vector

  // set the motor speed
  can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage4Byte, RoboteqNxtGenCANOpen::kMCSetMotorSpeedB1,
                      RoboteqNxtGenCANOpen::kMCSetMotorSpeedB2, RoboteqNxtGenCANOpen::kMCSetMotorSpeedSubIndex, rpm);
  return 0;
}

/**
 * @name RefreshReadings
 * @brief Read each value and return an error code if something went wrong
 * @return SocketCANOpen::kCANOpenSuccess, or error code from SocketCANOpen
 */
int RoboteqCleaningHeadMotor::RefreshReadings(int sequence)
{
  /**
   * @brief This function will refresh the non critical readings:
   * 1) Reads the heartbeat and exits if it has elapsed
   * 2) Read/Write the non critical measurements (RPM and current)
   */
  uint64_t now = can_socket_->GetCurrentTimeStamp();
  int status = 0;

  // Read heartbeat and process timeout
  uint8_t heartbeat_val;
  // 0ms timeout on reading the heartbeat since we don't actually want to wait for it
  status = can_socket_->ReadHeartbeatFromQueue(heartbeat_val, heartbeat_queue_, 0);
  if (status == SocketCANOpen::kCANOpenSuccess)
  {
    last_heartbeat_read_ = now;
    error_state_ = false;
    ROS_DEBUG_STREAM("Read heartbeat on motor #" << can_id_ << ": " <<
                     static_cast<int>(heartbeat_val));
  }
  if (now - last_heartbeat_read_ > RoboteqCleaningHeadMotor::kCANHeartbeatTimeoutMS)
  {
    error_state_ = true;
    ROS_WARN_STREAM("Heartbeat timeout on #" << can_id_);
    return RoboteqCleaningHeadMotor::kErrorHeartbeatTimeout;
  }

  // if, for whatever reason we're in an error state, don't try to read anything
  // until the hearbeat comes back
  if (error_state_)
  {
    return RoboteqCleaningHeadMotor::kErrorStateThrown;
  }


  // Write the RPM when sequence is equal to 1
  if (sequence == 1)
  {
    SetRPM();
  }

  // Read the amps on sequence 0
  if (sequence == 0) {
    // request data for Amps if sequence is 0
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadAmpB1,
                          RoboteqNxtGenCANOpen::kMCReadAmpB2, RoboteqNxtGenCANOpen::kMCReadAmpSubIndex);

    // wait one timeout for amps then either read or error
    uint16_t amps = 0;
    status = can_socket_->ReadSDOFromQueue(amps,
                                           amps_queue_,
                                           RoboteqCleaningHeadMotor::kCANReadTimeoutMS);
    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      amps_ = amps;
    }
    else
    {
      ROS_WARN_STREAM("Amps timeout on #" << can_id_);
      error_state_ = true;
      return RoboteqCleaningHeadMotor::kErrorReadValueTimeout;
    }
  }

  return RoboteqCleaningHeadMotor::kSuccess;
}

/**
 * @name SetManualOverride
 * @brief Enter, or exit manual override mode
 * @param manual_override true for manual override, false for normal operation
*/
void RoboteqCleaningHeadMotor::SetManualOverride(bool manual_override)
{
  // Doesn't do anything on brushed motors for now
}

