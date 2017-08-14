/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	sensor_safety_states_manager.cpp
 * @brief	Source File containing the SensorSafetyStatesManager class
 * @author	Feng Cao
 */

#include "avidbots_diagnostics/sensor_safety_states_manager.h"

/**
 * @name 	SensorSafetyStatesManager
 * @brief	Default constructor
 */
SensorSafetyStatesManager::SensorSafetyStatesManager()
{
}

/**
 * @name  ~SensorSafetyStatesManager
 * @brief Destructor
 */
SensorSafetyStatesManager::~SensorSafetyStatesManager() {}

/**
 * @name        Init()
 * @brief       Initialize SensorSafetyStatesManager
 * @param[in]   hardware_id, to identify different device
 */
void SensorSafetyStatesManager::Init(uint8_t hardware_id)
{
  ROS_INFO_STREAM("SensorSafetyStatesManager Init");
  id_ = hardware_id;
  num_states_ = avidbots_diagnostics_constants::SENSOR_SAFETY_MSG_ID_NUM;
  states_ = avidbots_msgs::diagnostics_states::kStateOk;
}

/**
 * @name        UpdateDiagnostics()
 * @brief       Update Diagnostics message
 * @param[in]   states, data reference for avidbots_msgs::diagnostics_states
 * @param[in]   diagnostics_msg, data reference for avidbots_msgs::diagnostics
 */
void SensorSafetyStatesManager::UpdateDiagnostics(const avidbots_msgs::diagnostics_states &states,
                                         avidbots_msgs::diagnostics &diagnostics_msg)
{
  diagnostics_msg.hardware_id = avidbots_msgs::diagnostics_states::kSensorSafety;
  diagnostics_msg.name = "Sensor Safety:\n";
  diagnostics_msg.status_sensor_safety = states.states;
  diagnostics_msg.stamp = ros::Time::now();
}
