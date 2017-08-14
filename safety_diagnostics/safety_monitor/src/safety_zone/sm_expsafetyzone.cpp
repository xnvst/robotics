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
 * @brief   Implementation file for SMExpSafetyZone class
 * @author  Keshav Iyengar
 */

// CPP
#include <vector>

// ROS
#include <ros/console.h>

// LOCAL
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/safety_zone/sm_expsafetyzone.h"

/**
 * @name        SMExpSafetyZone
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
SMExpSafetyZone::SMExpSafetyZone() {}

SMExpSafetyZone::~SMExpSafetyZone()
{}

/**
 * @name        Initialize
 * @brief       Initialize function
 */
void SMExpSafetyZone::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor Exponential Safety Zone ***");
  GetParam();
}

/**
 * @name  ZoneClear
 * @brief Checks if there are any points within the danger safety zone
 * @param[in] laser_cloud: The values of laser point cloud
 * @param[in] danger_zone_check: True if there are no points in the danger zone
 */
bool SMExpSafetyZone::ZoneClear(PointCloud::Ptr laser_cloud, bool zone_check)
{
  return CheckZonePointCloud(laser_cloud, zone_check, GetExpPolygonPoints());
}

/**
 * @name    CreateDangerSafetyZone()
 * @brief   Function to create points of polygon. Pushed into marker_ queue to be published in viz.
 * @param[in] velocity_vector: The velocity vector of the robot
 * @param[in] ang_vel: The angular velocity of the robot
 */
void SMExpSafetyZone::CreateExpSafetyZone(Point velocity_vector, double ang_vel)
{
  /* Clear last set of points and reset zones */
  ClearExpZoneMarker();
  ClearExpPolygonPoints();

  /* Initialize visualization for rviz */
  visualization_msgs::Marker exp_safety_zone_marker;

  exp_safety_zone_marker.header.frame_id = "/base_link";
  exp_safety_zone_marker.header.stamp = ros::Time();
  exp_safety_zone_marker.id = 0;
  exp_safety_zone_marker.type = visualization_msgs::Marker::LINE_LIST;
  exp_safety_zone_marker.action = visualization_msgs::Marker::ADD;

  /* Set marker scales */
  exp_safety_zone_marker.scale.x = .05;
  exp_safety_zone_marker.scale.y = .05;
  exp_safety_zone_marker.scale.z = .05;

  exp_safety_zone_marker.color.a = 1.0;
  exp_safety_zone_marker.color.r = 0.0;
  exp_safety_zone_marker.color.g = 1.0;
  exp_safety_zone_marker.color.b = 0.0;

  /* - Create normal vector for velocity.
   * - Cross product to determine which vertices are on side of velocity.
   * - Add velocity to those vertices.
   */

  /* Calculate velocity vector with constant. Apply a deadzone. */
  velocity_vector.x = (velocity_vector.x < 0.2 && velocity_vector.x > -0.2) ? (0) : (velocity_vector.x);
  velocity_vector.y = (velocity_vector.y < 0.2 && velocity_vector.y > -0.2) ? (0) : (velocity_vector.y);

  std::vector<Point> exp_polygon_points;

  /* Velocity expanding polygon. */
  exp_polygon_points = CreatePolygon(GetFootprintVertices(), GetZoneBaselineXAxisOffset());
  exp_polygon_points = Polygon::AddPolygonOffset(exp_polygon_points, kExpZoneOffset, GetZoneBaselineXAxisOffset());

  /* Do Cross Product and add in velocity for expanding safetyzone */
  exp_polygon_points = XProductCheckSide(velocity_vector, exp_polygon_points);

  ang_vel = (ang_vel < 0.1 && ang_vel > -0.1) ? (0) : (ang_vel);
  ang_vel = ang_vel * GetAngleVelConst();
  exp_polygon_points = Polygon::RotatePolygon(ang_vel, exp_polygon_points, GetZoneBaselineXAxisOffset());

  DrawPolygon(exp_polygon_points, exp_safety_zone_marker);
  SetExpZoneMarker(exp_safety_zone_marker);
  SetExpPolygonPoints(exp_polygon_points);
}
