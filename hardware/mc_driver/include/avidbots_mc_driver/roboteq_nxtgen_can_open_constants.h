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
 * @brief	Header File containing the constants related to the roboteq
          nxgen controllers
 * @author	Pablo Molina
 */

#ifndef AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_OPEN_CONSTANTS_H
#define AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_OPEN_CONSTANTS_H

#include <stdint.h>

namespace RoboteqNxtGenCANOpen
{

  const int kDelayBetweenCmdUS             = 10000;

  const uint32_t  kHeartBeat               = 0x700;

  // Object dictionary

  const uint8_t   kMCTempB1                = 0x21;
  const uint8_t   kMCTempB2                = 0x0F;

  const uint8_t   kMCReadAmpB1             = 0x21;
  const uint8_t   kMCReadAmpB2             = 0x00;
  const uint8_t   kMCReadAmpSubIndex       = 0x01;

  const uint8_t   kMCReadBattVoltsB1       = 0x21;
  const uint8_t   kMCReadBattVoltsB2       = 0x0D;
  const uint8_t   kMCReadBattVoltsSubIndex = 0x02;

  const uint8_t   kMCReadEncTickB1         = 0x21;
  const uint8_t   kMCReadEncTickB2         = 0x04;
  const uint8_t   kMCReadEncTickSubIndex1  = 0x01;
  const uint8_t   kMCReadEncTickSubIndex2  = 0x02;

  const uint8_t   kMCReadEncRPMB1          = 0x21;
  const uint8_t   kMCReadEncRPMB2          = 0x03;
  const uint8_t   kMCReadRPMSubIndex       = 0x01;

  const uint8_t   kMCReadBrlessTickB1      = 0x21;
  const uint8_t   kMCReadBrlessTickB2      = 0x05;

  const uint8_t   kMCSetMotorSpeedB1       = 0x20;
  const uint8_t   kMCSetMotorSpeedB2       = 0x00;
  const uint8_t   kMCSetMotorSpeedSubIndex = 0x01;

  const uint8_t   kMCEmergencyStopB1       = 0x20;
  const uint8_t   kMCEmergencyStopB2       = 0x0C;
  const uint8_t   kMCEmergencyStopSubIndex = 0x00;

  const uint8_t   kMCSetUserIntVarB1        = 0x20;
  const uint8_t   kMCSetUserIntVarB2        = 0x05;
  const uint8_t   kMCSetUserIntVar1SubIndex = 0x01;
  const uint8_t   kMCSetUserIntVar2SubIndex = 0x02;

  const uint8_t   kMCReleaseStopB1         = 0x20;
  const uint8_t   kMCReleaseStopB2         = 0x0D;
  const uint8_t   kMCReleaseStopSubIndex   = 0x00;

  const uint8_t   kMCCalibrateUserVarB1       = 0x21;
  const uint8_t   kMCCalibrateUserVarB2       = 0x06;
  const uint8_t   kMCCalibrateUserVarSubIndex = 0x01;

}  // namespace RoboteqNxtGenCANOpen

#endif  // AVIDBOTS_MC_DRIVER_ROBOTEQ_NXTGEN_CAN_OPEN_CONSTANTS_H
