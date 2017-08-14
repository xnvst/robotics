/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    sm_expsafetyzone
 * @brief   Header file for SMExpSafetyZone class
 * @author  Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_EXPSAFETYZONE_H
#define AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_EXPSAFETYZONE_H

// ROS
#include <ros/ros.h>

// LOCAL
#include <avidbots_library/safety_zone/properties.h>
#include "avidbots_safety_monitor/safety_zone/sm_safetyzone_base.h"

// MSGS
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/**
 * @name    SMExpSafetyZone
 */
class SMExpSafetyZone : public SMSafetyZoneBase
{
public:
  SMExpSafetyZone();
  ~SMExpSafetyZone();

  void Initialize();
  void CreateExpSafetyZone(Point velocity_vector, double ang_vel);
  bool ZoneClear(PointCloud::Ptr laser_cloud, bool zone_check);

private:
  double kExpZoneOffset = -0.05;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_EXPSAFETYZONE_H
