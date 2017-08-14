/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name  roboteq_drive_system_motor.cpp
 * @brief Source File containing the RRoboteQ drive system motor drivers class implementation
 * @author  Joseph Duchesne
 */

#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_mc_driver/roboteq_drive_system_motor.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can_open_constants.h"


/**
 * @name  Init
 * @brief Initializes the CAN device
 * @param[in] can_port: The CAN port to connect to
 */
int RoboteqDriveSystemMotor::Init(SocketCANOpenProtocol& can_socket, bool read_voltage)
{
  can_socket_ = &can_socket;

  non_critical_to_update_ = 3;
  if (read_voltage)
  {
    non_critical_to_update_++;
  }

  // assert if can_id_ hasn't been set
  ROS_DEBUG_STREAM("RoboteqBrushlessMotor::Init on can id " << can_id_);

  // set up listeners for things we're interested in

  // heartbeat
  struct CANOpenListenMask listen_mask =
      {RoboteqNxtGenCANOpen::kHeartBeat + can_id_, 0, {}};
  can_socket_->AddFrameListener(listen_mask, heartbeat_queue_);
  // don't timeout instantly, give it one interval
  last_heartbeat_read_ = can_socket_->GetCurrentTimeStamp();

  // Amps
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadAmpB2,
      RoboteqNxtGenCANOpen::kMCReadAmpB1, RoboteqNxtGenCANOpen::kMCReadAmpSubIndex
    }
  };
  can_socket_->AddFrameListener(listen_mask, amps_queue_);

  // RPM
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadEncRPMB2,
      RoboteqNxtGenCANOpen::kMCReadEncRPMB1, RoboteqNxtGenCANOpen::kMCReadRPMSubIndex
    }
  };
  can_socket_->AddFrameListener(listen_mask, RPM_queue_);

  // ticks
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage4Byte, RoboteqNxtGenCANOpen::kMCReadEncTickB2,
      RoboteqNxtGenCANOpen::kMCReadEncTickB1, RoboteqNxtGenCANOpen::kMCReadEncTickSubIndex1
    }
  };
  can_socket_->AddFrameListener(listen_mask, ticks_queue_);

  // voltage
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage2Byte,  RoboteqNxtGenCANOpen::kMCReadBattVoltsB2,
      RoboteqNxtGenCANOpen::kMCReadBattVoltsB1, RoboteqNxtGenCANOpen::kMCReadBattVoltsSubIndex
    }
  };
  can_socket_->AddFrameListener(listen_mask, voltage_queue_);

  read_voltage_ = read_voltage;

  return 0;
}


/**
 * @name  SetRPM
 * @brief Sets the RPM for the drive system motor
 */
int RoboteqDriveSystemMotor::SetRPM()
{
  if (error_state_)
  {
    return RoboteqDriveSystemMotor::kErrorStateThrown;
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
int RoboteqDriveSystemMotor::RefreshReadings(int sequence)
{
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
    ROS_DEBUG_STREAM("Read heartbeat on motor #" << can_id_ << ": " << heartbeat_val);
  }
  if (now - last_heartbeat_read_ > RoboteqDriveSystemMotor::kCANHeartbeatTimeoutMS)
  {
    error_state_ = true;
    ROS_WARN_STREAM("Heartbeat timeout on #" << can_id_);
    return RoboteqDriveSystemMotor::kErrorHeartbeatTimeout;
  }

  // if, for whatever reason we're in an error state, don't try to read anything
  // until the heartbeat comes back
  if (error_state_)
  {
    return RoboteqDriveSystemMotor::kErrorStateThrown;
  }

  ROS_DEBUG_STREAM("Reading sequence #" << sequence << " for motor " << can_id_);
  //  request data for Amps, RPM, ticks, voltage
  if (sequence == 0)
  {
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadAmpB1,
                          RoboteqNxtGenCANOpen::kMCReadAmpB2, RoboteqNxtGenCANOpen::kMCReadAmpSubIndex);
  }
  if (sequence == 1)
  {
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadEncRPMB1,
                        RoboteqNxtGenCANOpen::kMCReadEncRPMB2, RoboteqNxtGenCANOpen::kMCReadRPMSubIndex);
  }
  if (read_voltage_ && sequence == 2)
  {
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage2Byte, RoboteqNxtGenCANOpen::kMCReadBattVoltsB1,
                          RoboteqNxtGenCANOpen::kMCReadBattVoltsB2, RoboteqNxtGenCANOpen::kMCReadBattVoltsSubIndex);
  }

  // Always read ticks
  can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOReadMessage4Byte, RoboteqNxtGenCANOpen::kMCReadEncTickB1,
                        RoboteqNxtGenCANOpen::kMCReadEncTickB2, RoboteqNxtGenCANOpen::kMCReadEncTickSubIndex1);

  if (sequence == 0)
  {
    // wait one timeout for amps then either read or error
    uint16_t amps = 0;
    status = can_socket_->ReadSDOFromQueue(amps,
                                           amps_queue_,
                                           RoboteqDriveSystemMotor::kCANReadTimeoutMS);
    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      amps_ = amps;
    }
    else
    {
      error_state_ = true;
      ROS_WARN_STREAM("Amps reading timeout on #" << can_id_);
      return RoboteqDriveSystemMotor::kErrorReadValueTimeout;
    }
  }

  if (sequence == 1)
  {
    // wait one timeout for RPM then either read or error
    uint16_t RPM = 0;
    status = can_socket_->ReadSDOFromQueue(RPM,
                                           RPM_queue_,
                                           RoboteqDriveSystemMotor::kCANReadTimeoutMS);
    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      RPM_ = RPM;
    }
    else
    {
      error_state_ = true;
      ROS_WARN_STREAM("RPM reading timeout on #" << can_id_);
      return RoboteqDriveSystemMotor::kErrorReadValueTimeout;
    }
  }

  if (read_voltage_ && sequence == 2)
  {
    // wait one timeout for voltage then either read or error
    uint16_t voltage = 0;
    status = can_socket_->ReadSDOFromQueue(voltage,
                                           voltage_queue_,
                                           RoboteqDriveSystemMotor::kCANReadTimeoutMS);
    if (status == SocketCANOpen::kCANOpenSuccess)
    {
      voltage_ = voltage;
    }
    else
    {
      error_state_ = true;
      ROS_WARN_STREAM("Voltage reading timeout on #" << can_id_);
      return RoboteqDriveSystemMotor::kErrorReadValueTimeout;
    }
  }

  // wait one timeout for ticks then either read or error
  uint32_t ticks = 0;
  status = can_socket_->ReadSDOFromQueue(ticks,
                                         ticks_queue_,
                                         RoboteqDriveSystemMotor::kCANReadTimeoutMS);
  if (status == SocketCANOpen::kCANOpenSuccess)
  {
    ticks_ = ticks;
  }
  else
  {
    error_state_ = true;
    ROS_WARN_STREAM("Ticks reading timeout on #" << can_id_);
    return RoboteqDriveSystemMotor::kErrorReadValueTimeout;
  }


  return RoboteqDriveSystemMotor::kSuccess;
}

/**
 * @name SetManualOverride
 * @brief Enter, or exit manual override mode
 * @param manual_override true for manual override, false for normal operation
 */
void RoboteqDriveSystemMotor::SetManualOverride(bool manual_override)
{
  uint8_t value = 1;
  if (manual_override && !manual_override_)  // Enter manual override
  {
    // send EX (emergency shutdown) command to motor driver
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage1Byte, RoboteqNxtGenCANOpen::kMCEmergencyStopB1,
                    RoboteqNxtGenCANOpen::kMCEmergencyStopB2, RoboteqNxtGenCANOpen::kMCEmergencyStopSubIndex, value);
    ROS_INFO_STREAM("MC Driver: Motor #" << can_id_ << " manual override enabled");
  }
  else if (!manual_override && manual_override_)  // Exit manual override
  {
    // send MG (release shutdown) command to motor driver
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage1Byte, RoboteqNxtGenCANOpen::kMCReleaseStopB1,
                RoboteqNxtGenCANOpen::kMCReleaseStopB2, RoboteqNxtGenCANOpen::kMCReleaseStopSubIndex, value);
    ROS_INFO_STREAM("MC Driver: Motor #" << can_id_ << " manual override disabled");
  }

  last_manual_override_ = can_socket_->GetCurrentTimeStamp();

  // store the current value
  manual_override_ = manual_override;
}

/**
 * @name  SetShutdown
 * @brief Set or Release Shutdown for the drive system motor
 */
int RoboteqDriveSystemMotor::SetShutdown(bool on)
{
  uint8_t value = 0;
  if (on)
  {
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage1Byte, SocketCANOpen::kEmergencyShutdownB1,
                      SocketCANOpen::kEmergencyShutdownB2, SocketCANOpen::kEmergencyShutdownIndex, value);
  }
  else
  {
    can_socket_->SendSDO(can_id_, SocketCANOpen::kSDOWriteMessage1Byte, SocketCANOpen::kReleaseShutdownB1,
                      SocketCANOpen::kReleaseShutdownB2, SocketCANOpen::kReleaseShutdownIndex, value);
  }
  return 0;
}

