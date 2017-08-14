
/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  roboteq_steer_system_motor.cpp
 * @brief Source File containing the RRoboteQ steer system motor drivers class implementation
 * @author  Joseph Duchesne, Yoohee Choi
 */

// Standard
#include <stdint.h>

// Boost
#include <boost/lockfree/spsc_queue.hpp>

#include "avidbots_library/socket_can/socket_can_open_constants.h"
#include "avidbots_mc_driver/roboteq_steer_system_motor.h"
#include "avidbots_mc_driver/roboteq_nxtgen_can_open_constants.h"

/**
 * @name 	Init
 * @brief	Initializes the CAN device
 * @param[in] can_socket: The CAN port to connect to
 * @param[in] read_voltage: flag to enable voltage reading
 */
int RoboteqSteerSystemMotor::Init(SocketCANOpenProtocol& can_socket,
                                  bool read_voltage)
{
  RoboteqDriveSystemMotor::Init(can_socket, read_voltage);

  /* Steer system requires a voltage reading + calibration reading */
  non_critical_to_update_ = 4;

  read_calibration_status_ = true;

  /* Set up listeners for calibration */
  // calibration
  struct CANOpenListenMask listen_mask;
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage4Byte,
      RoboteqNxtGenCANOpen::kMCCalibrateUserVarB2,
      RoboteqNxtGenCANOpen::kMCCalibrateUserVarB1,
      RoboteqNxtGenCANOpen::kMCCalibrateUserVarSubIndex
    }
  };
  can_socket_->AddFrameListener(listen_mask, calibration_queue_);

  return 0;
}

/**
 * @name RefreshReadings
 * @brief Read each value and return an error code if something went wrong
 * @return SocketCANOpen::kCANOpenSuccess, or error code from SocketCANOpen
 */
int RoboteqSteerSystemMotor::RefreshReadings(int sequence)
{
  int status = RoboteqDriveSystemMotor::RefreshReadings(sequence);
  if (status != RoboteqDriveSystemMotor::kSuccess)
  {
    return status;
  }

  // Read calibration status
  /* Calibration status doesn't need to be read as frequently
     as the other readings.
     Skipping calibration status check every other time. */
  if (sequence == 3)
  {
    if (read_calibration_status_)
    {
      can_socket_->SendSDO(can_id_,
                           SocketCANOpen::kSDOReadMessage4Byte,
                           RoboteqNxtGenCANOpen::kMCCalibrateUserVarB1,
                           RoboteqNxtGenCANOpen::kMCCalibrateUserVarB2,
                           RoboteqNxtGenCANOpen::kMCCalibrateUserVarSubIndex);

      // Wait one timeout before reading
      // Calibration status from controller: 0 during calibration; 1 when
      // not calibrating.
      uint32_t calibrated = 0;
      status = can_socket_->ReadSDOFromQueue(calibrated,
                    calibration_queue_,
                    RoboteqDriveSystemMotor::kCANReadTimeoutMS);
      if (status == SocketCANOpen::kCANOpenSuccess)
      {
        ROS_DEBUG_STREAM_THROTTLE(1.0, "Calibration status: " << calibrated);
        calibration_ = (calibrated == 0);
      }
      else
      {
        error_state_ = true;
        ROS_WARN_STREAM("Calibration reading timeout on #" << can_id_);
        return RoboteqDriveSystemMotor::kErrorReadValueTimeout;
      }
    }

    read_calibration_status_ = !read_calibration_status_;
  }

  return RoboteqDriveSystemMotor::kSuccess;
}
