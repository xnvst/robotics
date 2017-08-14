/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	avidbots_diagnostics_constants.h
 * @brief	Header file containing the avidbots_diagnostics_constants namespace
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_CONSTANTS_H
#define AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_CONSTANTS_H

namespace avidbots_diagnostics_constants
{
  // Driver update diagnostics frequency, in sec
  // set to 50Hz to capture change faster
  const uint8_t UPDATE_FREQ = 0.02;

  // Diagnostics msg id
  enum MCUMsgID
  {
    MCU_CONNECTION_STATUS,
    MCU_EXIT_DRIVER_LOOP,
    MCU_STATE_VARIABLE,
    MCU_MSG_ID_NUM
  };
  enum MCMsgID
  {
    MC_WHEEL,
    MC_STEERING,
    EXIT_DRIVER_LOOP,
    MC_MSG_ID_NUM
  };
  enum URGMsgID
  {
    URG_HARDWARE_STATUS = 0,
    HOKUYO_LASER_SCAN_STATUS = 0,
    URG_LASER_SCAN_STATUS = 1,
    HOKUYO_HARDWARE_STATUS = 1,
    URG_MSG_ID_NUM
  };
  enum CameraMsgID
  {
    CAMERA_FREQ_STATUS,
    CAMERA_MSG_ID_NUM
  };
  enum SensorSafetyMsgID
  {
    SENSOR_SAFETY_LASERS,
    SENSOR_SAFETY_ODOMETRY,
    SENSOR_SAFETY_ENCODER,
    SENSOR_SAFETY_CAMERA_LEFT,
    SENSOR_SAFETY_CAMERA_RIGHT,
    SENSOR_SAFETY_MSG_ID_NUM
  };

  enum LaserType
  {
    LASER_NONE,
    LASER_URG,
    LASER_HOKUYO,
  };
}   // namespace avidbots_diagnostics_constants

#endif  // AVIDBOTS_DIAGNOSTICS_AVIDBOTS_DIAGNOSTICS_CONSTANTS_H
