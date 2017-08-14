/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  Safety monitor states
 * @brief Header file describing state type
 * @author  Feng Cao
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SM_CONSTANTS_H
#define AVIDBOTS_SAFETY_MONITOR_SM_CONSTANTS_H

/**
 * @brief   States for state machine
 */
typedef enum
{
  INIT_STATE,            // 0
  IDLE_STATE,            // 1
  DIAGNOSTICS_STATE,     // 2
  DANGERZONE_STATE,      // 3
  EXP_SAFETYZONE_STATE,  // 4
  LOW_VOLTAGE_STATE,     // 5
  ESTOP_STATE            // 6
}
States;

/**
 * @brief   State Transition Events
 */
typedef enum
{
  INIT_EVENT,
  DIAGNOSTICS_EVENT,
  DANGERZONE_EVENT,
  EXP_SAFETYZONE_EVENT,
  LOW_VOLTAGE_EVENT,
  ESTOP_EVENT,
  FAILURE_RESET_EVENT,
}
Events;

typedef enum
{
  ERROR,      // 0 (false)
  NO_ERROR    // 1 (true)
}
ErrorStatus;

#define NODE_START_DELAY               10
#define DIAGNOSTICS_START_DELAY        2
#define DIAGNOSTICS_TIMEOUT            0

// from ui_library.h
typedef enum
{
  STATE_LOADING,
  STATE_LOGIN,
  STATE_MAIN,
  STATE_MANUAL,
  STATE_PLAN,
  STATE_MOVE,
  STATE_LOAD_CLEAN,
  STATE_CLEAN,
  STATE_PAUSE,
  STATE_OBSTACLE,
  STATE_PROCESSING,
  STATE_GOING_HOME,
  STATE_ESTOP,
  STATE_OFF,
  STATE_DISCONNECTION,
  STATE_DANGER_OBSTACLE,
  STATE_MOVE_LEARN,
  STATE_LEARN,
  STATE_SM_FAILURE,
  STATE_LOGIN_CHECKLIST,
  STATE_LOGOUT_CHECKLIST,
  STATE_MAPPING,
  STATE_SYNC,
  STATE_LOGOUT,
  STATE_WATER_FAILURE,
  STATE_REMOTE_PAUSE,
  STATE_REMOTE_DETOUR,
  STATE_ESTOP_DELAY,
  STATE_AUTO_SYNC,
  STATE_DIAGNOSTICS
}
UIStates;

#endif  // AVIDBOTS_SAFETY_MONITOR_SM_CONSTANTS_H
