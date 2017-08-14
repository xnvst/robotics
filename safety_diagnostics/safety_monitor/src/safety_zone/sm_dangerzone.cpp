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
 * @brief   Implementation file for SMDangerZone class
 * @author  Keshav Iyengar
 */

// CPP
#include <vector>

// ROS
#include <ros/console.h>

// LOCAL
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/safety_zone/sm_dangerzone.h"

/**
 * @name        SMDangerZone
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
SMDangerZone::SMDangerZone() {}

SMDangerZone::~SMDangerZone()
{}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void SMDangerZone::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor Danger Zone ***");
  GetParam();
}

/**
 * @name  ZoneClear
 * @brief Checks if there are any points within the danger safety zone
 * @param[in] laser_cloud: The values of laser point cloud
 * @param[in] danger_zone_check: True if there are no points in the danger zone
 */
bool SMDangerZone::ZoneClear(PointCloud::Ptr laser_cloud, bool zone_check)
{
  return CheckZonePointCloud(laser_cloud, zone_check, GetDangerPolygonPoints());
}


/**
 * @name    CreateDangerSafetyZone()
 * @brief   Function to create points of polygon. Pushed into marker_ queue to be published in viz.
 */
void SMDangerZone::CreateDangerZone()
{
  /* Clear last set of points and reset zones */
  ClearDangerZoneMarker();
  ClearDangerPolygonPoints();

  /* Initialize visualization for rviz */
  visualization_msgs::Marker danger_zone_marker;

  danger_zone_marker.header.frame_id = "/base_link";
  danger_zone_marker.header.stamp = ros::Time();
  danger_zone_marker.id = 0;
  danger_zone_marker.type = visualization_msgs::Marker::LINE_LIST;
  danger_zone_marker.action = visualization_msgs::Marker::ADD;

  /* Set marker scales */
  danger_zone_marker.scale.x = .05;
  danger_zone_marker.scale.y = .05;
  danger_zone_marker.scale.z = .05;

  danger_zone_marker.color.a = 1.0;
  danger_zone_marker.color.r = 0.0;
  danger_zone_marker.color.g = 0.0;
  danger_zone_marker.color.b = 1.0;


  /* Danger zone polygon */
  std::vector<Point> danger_polygon_points;
  danger_polygon_points = CreatePolygon(GetFootprintVertices(),
                                        GetZoneBaselineXAxisOffset());

  /* Create danger zone in rviz */
  DrawPolygon(danger_polygon_points, danger_zone_marker);
  SetDangerZoneMarker(danger_zone_marker);
  SetDangerPolygonPoints(danger_polygon_points);
}
