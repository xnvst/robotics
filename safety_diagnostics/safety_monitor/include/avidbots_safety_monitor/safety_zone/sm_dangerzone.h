/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    sm_dangerzone
 * @brief   Header file for SMDangerZone class
 * @author  Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_DANGERZONE_H
#define AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_DANGERZONE_H

// ROS
#include <ros/ros.h>

// LOCAL
#include <avidbots_library/safety_zone/properties.h>
#include "avidbots_safety_monitor/safety_zone/sm_safetyzone_base.h"

// MSGs
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @name    SMDangerZone
 */
class SMDangerZone : public SMSafetyZoneBase
{
public:
  SMDangerZone();
  ~SMDangerZone();

  void Initialize();
  void CreateDangerZone();
  bool ZoneClear(PointCloud::Ptr laser_cloud, bool zone_check);

private:
};

#endif  // AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_DANGERZONE_H
