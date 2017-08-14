/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  wheely_mcu_driver.h
 * @brief Header File containing the WheelyMCUDriver class
 * @author  Jake Park
 */

#ifndef AVIDBOTS_MCU_DRIVER_WHEELY_MCU_DRIVER_H
#define AVIDBOTS_MCU_DRIVER_WHEELY_MCU_DRIVER_H

#include "avidbots_mcu_driver/mcu_driver.h"
#include "avidbots_msgs/water_sensor_status.h"

#include "std_msgs/Int32.h"

class WheelyMCUDriver : public AvidbotsMCUDriver
{
private:
  uint8_t dirty_water_full_state_  = 0;
  uint8_t clean_water_empty_state_ = 0;

  uint8_t ver_major_ = 0;
  uint8_t ver_minor_ = 0;
  uint8_t ver_patch_ = 0;

  uint16_t steering_val_ = 0;
  uint16_t driving_val_ = 0;

  int mcu_version_ok_ = -1;
  uint8_t mcu_version_cnt_ = 0;
  const uint8_t MAX_MCU_VERSION_CNT_ = 100;

  ros::Publisher water_sensor_status_pub_;

  avidbots_msgs::water_sensor_status water_sensor_status_msg_;

protected:
  void ManagePubAndSub();
  void GetParamSettings(const ros::NodeHandle& private_nh);
  int ManageMCUVersion(void);
  int ManageManualMotion(void);
  int ManageStatusVariable(void);
  int ManagePendingMessages(void);
  int InitDriver(const ros::NodeHandle &private_nh, ros::NodeHandle *node_handle);

public:
  void StartPublishStatusLoop();
};

#endif  // AVIDBOTS_MCU_DRIVER_WHEELY_MCU_DRIVER_H
