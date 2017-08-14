/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name  Sensor Safety Constants
 * @brief Header file describing state type
 * @author Paul Duchesne
 */

#ifndef AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_CONSTANTS_H
#define AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_CONSTANTS_H

typedef enum
{
  SENSOR_OK,         // 0
  SENSOR_ERROR,      // 1
}
SENSOR_STATUS;

const double FREQ_10HZ = 0.1;
const double FREQ_15HZ = 0.066;
const double FREQ_20HZ = 0.05;
const double FREQ_40HZ = 0.025;


#endif  // AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_CONSTANTS_H
