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
 * @brief	Source file containing the Avidbots CAN open implementation
 * @author	Pablo Molina
 */
#include <string>
#include <vector>
#include "avidbots_mcu_driver/avidbots_can_open.h"
#include "avidbots_library/socket_can/avidbots_can_open_constants.h"
#include "avidbots_msgs/mcu_output_cmd.h"

static const int kUnknownMCUOutputCommand = -1;

/**
 * @name 	Default constructor
 */
AvidbotsCANOpenController::AvidbotsCANOpenController()
{
  can_id_ = 0;
}

/**
 * @name 	Init
 * @brief	This function initializes the CAN device
 * @param[in] string can_port: The CAN port to connect to
 * @param[in] timeout_read_ms: The timeout for CAN commands
 * @param[in] can_id: The CANID of the device to connect to
 */
int AvidbotsCANOpenController::Init(const std::string &portName,
                                const int &timeout_read_ms,
                                const uint16_t &can_id)
{
  can_id_ = can_id;

  int return_value = can_socket_.InitSocket(portName, timeout_read_ms);

  // set up listening queues for each command that we're interested in reading
  struct CANOpenListenMask listen_mask;

  // Status Variables lister queue
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage1Byte,  kReadStatusVariableB2,
      kReadStatusVariableB1, kReadStatusVarSubIndex
    }
  };
  can_socket_.AddFrameListener(listen_mask, status_variable_queue_);

  // Manual Motion lister queue
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage4Byte,  kManualMotionCmdB2,
      kManualMotionCmdB1, kManualMotionIndex
    }
  };
  can_socket_.AddFrameListener(listen_mask, manualmotion_variable_queue_);

  // MCU version lister queue
  listen_mask = { SocketCANOpen::kSDOReceive + can_id_, 4,
    {
      SocketCANOpen::kSDOReadMessage4Byte,  kVersionCmdB2,
      kVersionCmdB1, kVersionSubIndex
    }
  };
  can_socket_.AddFrameListener(listen_mask, mcu_version_queue_);

  // Zero values
  for (int i = 0; i < avidbots_msgs::mcu_output_cmd::kOutputCount; i++)
  {
    unconfirmed_writes_[i] = false;
  }

  return return_value;
}

/**
 * @name 	ProcessOutputCommand
 * @brief	This function processes a process output command
 * @param[in] output_num: The device to output to
 * @param[in] output_val: The value to output to it
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::ProcessOutputCommand(const uint8_t& output_num,
                         const uint8_t& output_val)
{
  ROS_DEBUG_STREAM("MCU Driver: Recieved message #"
    << static_cast<int>(output_num) << ": " << static_cast<int>(output_val));

  if (output_num < 0 || output_num >= avidbots_msgs::mcu_output_cmd::kOutputCount)
  {
    ROS_ERROR_STREAM("*** Unknown output_num in AvidbotsCANOpenController::ProcessOuputCommand *** :"
      << static_cast<int>(output_num));
    return kUnknownMCUOutputCommand;
  }

  // Copy the data into the structure
  unconfirmed_writes_[output_num] = true;
  unconfirmed_write_data_[output_num] = output_val;
  return 0;
}

/**
 * @name 	WritePendingMessages
 * @brief	Writes the pending commands to the CAN socket
 */
int AvidbotsCANOpenController::WritePendingMessages()
{
  int ret = 0;
  // Write any pending messages
  for (int i = 0; i < avidbots_msgs::mcu_output_cmd::kOutputCount; i++)
  {
    if (unconfirmed_writes_[i])
    {
      unconfirmed_writes_[i] = false;
      ret += WriteOutputCommand(i, unconfirmed_write_data_[i]);
      usleep(kCANWaitBetweenWriteUS);
      ROS_DEBUG_STREAM("MCU Driver: Writing out #" << i << ": "
                       << static_cast<int>(unconfirmed_write_data_[i]));
    }
  }
  return ret;
}

/**
 * @name 	GetOutputCommandForCANRequest
 * Pass in a can frame, return the output command
 * @param  can_frame An incoming CAN frame
 * @return           Int avidbots_msgs::mcu_output_cmd or -1
 */
int AvidbotsCANOpenController::GetOutputCommandForCANRequest(struct can_frame frame)
{
  // Vacuum
  if (frame.data[1] == kVacuumCmdB2 && frame.data[2] == kVacuumCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kVacuum;
  }
  else if (frame.data[1] == kBrushCmdB2 && frame.data[2] == kBrushCmdB1)
  {
    if (frame.data[3] == kMainBrushSubIndex) {
      return avidbots_msgs::mcu_output_cmd::kMainBrush;
    }
    else
    {
      return avidbots_msgs::mcu_output_cmd::kSideBrush;
    }
  }
  else if (frame.data[1] == kCleaningHeadCmdB2 && frame.data[2] == kCleaningHeadCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kCleaningHead;
  }
  else if (frame.data[1] == kLEDStatusCmdB2 && frame.data[2] == kLEDStatusCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kLEDStatus;
  }
  else if (frame.data[1] == kPumpSpeedCmdB2 && frame.data[2] == kPumpSpeedCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kPump;
  }
  else if (frame.data[1] == kCancelEStopCmdB2 && frame.data[2] == kCancelEStopCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kCancelEStop;
  }
  else if (frame.data[1] == kLEDDirCmdB2 && frame.data[2] == kLEDDirCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kLEDDir;
  }
  else if (frame.data[1] == kBrakeCmdB2 && frame.data[2] == kBrakeCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kBrake;
  }
  else if (frame.data[1] == kSafetyMonitorStatusB2 &&
           frame.data[2] == kSafetyMonitorStatusB1)
  {
    return avidbots_msgs::mcu_output_cmd::kSafetyMonitor;
  }
  else if (frame.data[1] == kMCModeCmdB2 &&
             frame.data[2] == kMCModeCmdB1)
  {
    return avidbots_msgs::mcu_output_cmd::kDrivingMode;
  }

  ROS_ERROR_STREAM("Unknown: " << static_cast<int>(frame.data[1]) << ", "
    << static_cast<int>(frame.data[2]) << ", " << static_cast<int>(frame.data[3]));

  return kUnknownMCUOutputCommand;
}

/**
 * @name  WriteOutputCommand
 * @brief This function processes a process output command
 * @param[in] output_num: The device to output to
 * @param[in] output_val: The value to output to it
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::WriteOutputCommand(const uint8_t& output_num,
                         const uint8_t& output_val)
{
  switch (output_num)
  {
    case avidbots_msgs::mcu_output_cmd::kVacuum:
      return SetVacuumState(output_val);
    case avidbots_msgs::mcu_output_cmd::kMainBrush:
      return SetMainBrushState(output_val);
    case avidbots_msgs::mcu_output_cmd::kSideBrush:
      return SetSideBrushState(output_val);
    case avidbots_msgs::mcu_output_cmd::kCleaningHead:
      return SetCleaningHeadPosition(output_val);
    case avidbots_msgs::mcu_output_cmd::kLEDStatus:
      return SetLEDStatus(output_val);
    case avidbots_msgs::mcu_output_cmd::kPump:
      return SetPumpSpeed(output_val);
    case avidbots_msgs::mcu_output_cmd::kBrake:
      return SetBrake(output_val);
    case avidbots_msgs::mcu_output_cmd::kLEDDir:
      return SetLEDDirection(output_val);
    case avidbots_msgs::mcu_output_cmd::kCancelEStop:
      return SetCancelEStop(output_val);
    case avidbots_msgs::mcu_output_cmd::kSafetyMonitor:
      return SetSafetyMonitorStatus(output_val);
    case avidbots_msgs::mcu_output_cmd::kDrivingMode:
      return SetDrivingMode(output_val);
    default:
      ROS_ERROR_STREAM("*** Unknown output_num in AvidbotsCANOpenController::ProcessOuputCommand ***");
      break;
  }
  usleep(kCANWaitBetweenReadsUS);
  return -1;
}

/**
 * @name 	SetVacuumState
 * @brief	This function set the vacuum value
 * @param[in] vacuum_state: The vacuum value
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetVacuumState(uint8_t vacuum_state)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kVacuumCmdB1, kVacuumCmdB2,
                             kVacuumSubIndex, vacuum_state);
}

/**
 * @name 	SetMainBrushState
 * @brief	This function set the main brush value
 * @param[in] brush_state: The main brush value
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetMainBrushState(uint8_t brush_state)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kBrushCmdB1, kBrushCmdB2,
                             kMainBrushSubIndex, brush_state);
}

/**
 * @name 	SetSideBrushState
 * @brief	This function set the side brush value
 * @param[in] brush_state: The side brush value
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetSideBrushState(uint8_t brush_state)
{
  return can_socket_.SendSDO(can_id_,
                            SocketCANOpen::kSDOWriteMessage1Byte,
                            kBrushCmdB1, kBrushCmdB2,
                            kSideBrushSubIndex, brush_state);
}

/**
 * @name 	SetCleaningHeadPosition
 * @brief	This function set the cleaning head position
 * @param[in] brush_state: The cleaning head position
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetCleaningHeadPosition(uint8_t ch_pos)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kCleaningHeadCmdB1, kCleaningHeadCmdB2,
                             kCleaningHeadSubIndex, ch_pos);
}

/**
 * @name 	SetLEDStatus
 * @brief	This function sets the LED status
 * @param[in] cleaning_mode: The cleaning mode
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetLEDStatus(uint8_t led_status)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kLEDStatusCmdB1, kLEDStatusCmdB2,
                             kLEDStatusSubIndex, led_status);
}

/**
 * @name 	SetPumpSpeed
 * @brief	This function sets the pump speed
 * @param[in] pump_speed: The pump speed
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetPumpSpeed(uint8_t pump_speed)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kPumpSpeedCmdB1, kPumpSpeedCmdB2,
                             kPumpSpeedSubIndex, pump_speed);
}

/**
 * @name  SetCancelEStop
 * @brief This function cancels the estop status.
 *        It will turn back on if it's still being triggered.
 * @param[in] value: Should be 1 to disable e-stop status.
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetCancelEStop(uint8_t value)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kCancelEStopCmdB1, kCancelEStopCmdB2,
                             kCancelEStopSubIndex, value);
}

/**
 * @name  SetLEDDirection
 * @brief This function sends the LED directional command
 * @param[in] value: The LED directional values
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetLEDDirection(uint8_t led_dir)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kLEDDirCmdB1, kLEDDirCmdB2,
                             kLEDDirSubIndex, led_dir);
}

/**
 * @name  SetBrake
 * @brief This function sends the brake command
 * @param[in] value: The brake value
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetBrake(uint8_t value)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kBrakeCmdB1, kBrakeCmdB2,
                             kBrakeSubIndex, value);
}

/**
 * @name  SetSafetyMonitorStatus
 * @brief This function sends the safety monitor message
 * @param[in] value: The status of safety monitor
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::SetSafetyMonitorStatus(uint8_t value)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kSafetyMonitorStatusB1,
                             kSafetyMonitorStatusB2,
                             kSafetyMonitorStatusSubIndex, value);
}

int AvidbotsCANOpenController::SetDrivingMode(uint8_t value)
{
  return can_socket_.SendSDO(can_id_,
                             SocketCANOpen::kSDOWriteMessage1Byte,
                             kMCModeCmdB1,
                             kMCModeCmdB2,
                             kMCModeSubIndex,
                             value);
}

/**
 * @name  GetStatusVariable
 * @brief This function returns the state variable of the robot
 * @param[out] uint8_t  state_variable: The state variable of the robot 
 * @param[out] uint8_t  estop_status: The E-Stop status
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::GetStatusVariable(uint8_t& state_variable, uint8_t& estop_status)
{
  // Request the value
  can_socket_.SendSDO(can_id_,
                      SocketCANOpen::kSDOReadMessage1Byte,
                      kReadStatusVariableB1, kReadStatusVariableB2,
                      kReadStatusVarSubIndex);

  usleep(kCANWaitBetweenReadsUS);
  // Read from queue
  uint8_t value;
  int status = can_socket_.ReadSDOFromQueue(value,
                                            status_variable_queue_,
                                            kCANReadTimeoutMS);

  ROS_DEBUG_STREAM("CAN controller read status: " << status);
  if (status != SocketCANOpen::kCANOpenSuccess)
  {
    ROS_WARN_STREAM("MCU read status variable timeout on #" << can_id_);
    return AvidbotsCANOpenController::kErrorReadValueTimeout;
  }
  else
  {
    MCUDecomposeStatusMsg(&estop_status, &state_variable, value);
    ROS_DEBUG("MCU state variable, value: %x, estop_status: %x, state_var: %x", value,
                                                                                estop_status,
                                                                                state_variable);
  }
  return AvidbotsCANOpenController::kSuccess;
}

/**
 * @name  GetStatusVariable
 * @brief This function returns the state variable of the robot
 *        along with the states of the clean and dirty water sensors
 * @param[out] uint8_t  clean_empty_state: The state variable for the
 *                                         clean water tank sensor
 * @param[out] uint8_t  dirty_full_state:  The state variable for
 *                                         the dirty water tank sensor
 * @param[out] uint8_t  state_variable:    The state variable of the robot
 * @param[out] uint8_t  estop_status:      The E-Stop status
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::GetStatusVariable(uint8_t &clean_empty_state,
                                                 uint8_t &dirty_full_state,
                                                 uint8_t &estop_status,
                                                 uint8_t &state_variable)
{
  // Request the value
  can_socket_.SendSDO(can_id_,
                      SocketCANOpen::kSDOReadMessage1Byte,
                      kReadStatusVariableB1, kReadStatusVariableB2,
                      kReadStatusVarSubIndex);

  usleep(kCANWaitBetweenReadsUS);
  // Read from queue
  uint8_t value;
  int status = can_socket_.ReadSDOFromQueue(value,
                                            status_variable_queue_,
                                            kCANReadTimeoutMS);

  ROS_DEBUG_STREAM("CAN controller read status: " << status);
  if (status != SocketCANOpen::kCANOpenSuccess)
  {
    ROS_WARN_STREAM("MCU read status variable timeout on #" << can_id_);
    return AvidbotsCANOpenController::kErrorReadValueTimeout;
  }
  else
  {
    MCUDecomposeStatusMsg(&clean_empty_state,
                          &dirty_full_state,
                          &estop_status,
                          &state_variable,
                          value);
    /* Reverting clean_empty_state as the physical switch has been flipped. */
    if (clean_empty_state == 0)
      clean_empty_state = 1;
    else
      clean_empty_state = 0;

    ROS_DEBUG("MCU state variable, value: %x, estop_status: %x,"
              "state_var: %x dirty_full_state: %x, clean_empty_state %x",
              value,
              estop_status,
              state_variable,
              dirty_full_state,
              clean_empty_state);
  }
  return AvidbotsCANOpenController::kSuccess;
}

/**
 * @name  GetManualMotionVariable
 * @brief This function returns the motion variable of the robot
 * @param[out] uint16_t  steering_val 
 * @param[out] uint16_t  driving_val
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::GetManualMotionVariable(uint16_t& steering_val, uint16_t& driving_val)
{
  // Request the value
  can_socket_.SendSDO(can_id_,
                      SocketCANOpen::kSDOReadMessage4Byte,
                      kManualMotionCmdB1, kManualMotionCmdB2,
                      kManualMotionIndex);

  usleep(kCANWaitBetweenReadsUS);

  // Read from queue
  uint32_t value = 0;
  int status = can_socket_.ReadSDOFromQueue(value,
                                            manualmotion_variable_queue_,
                                            kCANReadTimeoutMS);

  if (status != SocketCANOpen::kCANOpenSuccess)
  {
    ROS_WARN_STREAM("MCU read ManualMotion variable timeout on #" << can_id_);
    return AvidbotsCANOpenController::kErrorReadValueTimeout;
  }
  else
  {
    MCUDecomposeSteerDriveMsg(&steering_val, &driving_val, reinterpret_cast<uint8_t *>(&value));
    if (value)
    {
        ROS_DEBUG("MCU ManualMotion variable, value: 0x%x, steering_val: 0x%x, driving_val: 0x%x", value,
                                                                                steering_val,
                                                                                driving_val);
    }
  }
  return AvidbotsCANOpenController::kSuccess;
}

/**
 * @name  GetMCUFirmwareVersion
 * @brief This function returns the MCU Firmware Version
 * @param[out] uint8_t  major 
 * @param[out] uint8_t  minor
 * @param[out] uint8_t  patch
 * @return The status of the current command sent
 */
int AvidbotsCANOpenController::GetMCUFirmwareVersion(uint8_t& major, uint8_t& minor, uint8_t& patch)
{
  // Request the value
  can_socket_.SendSDO(can_id_,
                      SocketCANOpen::kSDOReadMessage4Byte,
                      kVersionCmdB1, kVersionCmdB2,
                      kVersionSubIndex);

  usleep(kCANWaitBetweenReadsUS);

  // Read from queue
  uint32_t value = 0;
  int status = can_socket_.ReadSDOFromQueue(value,
                                            mcu_version_queue_,
                                            kCANReadTimeoutMS);

  if (status != SocketCANOpen::kCANOpenSuccess)
  {
    ROS_WARN_STREAM("MCU read MCU Firmware Version timeout on #" << can_id_);
    return AvidbotsCANOpenController::kErrorReadValueTimeout;
  }
  else
  {
    MCUDecomposeVersionMsg(&major, &minor, &patch, reinterpret_cast<uint8_t *>(&value));
    ROS_DEBUG("MCU Firmware Version: 0x%x, %d.%d.%d\n", value, major, minor, patch);
  }
  return AvidbotsCANOpenController::kSuccess;
}

/**
 * @name    ClosePort
 * @brief   Closes the CAN port
 */
void AvidbotsCANOpenController::CloseSocket()
{
  can_socket_.CloseSocket();
}
