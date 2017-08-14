/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    state_base
 * @brief   Header file for SMSafetyZoneBase class
 * @author  Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_SAFETYZONE_BASE_H
#define AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_SAFETYZONE_BASE_H

// CPP
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

// LOCAL
#include <avidbots_library/safety_zone/polygon.h>
#include <avidbots_library/safety_zone/properties.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @name    SMSafetyZoneBase
 * @brief   Base classes for exp safety zone and danger zone class
 */
class SMSafetyZoneBase
{
public:
  SMSafetyZoneBase();
  virtual ~SMSafetyZoneBase() {}
  // Common function between safety zone clasess
  virtual void Initialize() {}
  virtual void CreateExpSafetyZone(Point velocity_vector, double ang_vel) {}
  virtual void CreateDangerZone() {}
  virtual bool ZoneClear(PointCloud::Ptr laser_cloud, bool zone_check) {}

  void InitLaserInfo();
  visualization_msgs::Marker GetDangerZoneMarker() const;
  visualization_msgs::Marker GetExpZoneMarker() const;

protected:
  /* ROS Object */
  ros::NodeHandle                  node_;

  // Common functions to execute
  // From helper class
  void GetParam();
  double StoppingDistance(Point vector, double accel, double multiplier = 1);
  std::vector<Point> CreatePolygon(geometry_msgs::PolygonStamped footprint_vertices, double baseline_x_axis_offset);

  bool PinPolygon(Point &point, std::vector<Point> polygon_point, bool &safety_zone);
  bool WaitForLaserTransform(const sensor_msgs::LaserScan::ConstPtr& scan);
  PointCloud::Ptr GetPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);

  void DrawPolygon(std::vector<Point> &polygon_points, visualization_msgs::Marker &zone_marker);
  std::vector<Point> XProductCheckSide(Point &vector, std::vector<Point> polygon_points);
  bool CheckZonePointCloud(PointCloud::Ptr laser_cloud,
                                 bool safety_zone_check,
                                 std::vector<Point> polygon_points);

  std::vector<Point> GetExpPolygonPoints() const;
  std::vector<Point> GetDangerPolygonPoints() const;
  geometry_msgs::PolygonStamped GetFootprintVertices() const;
  double GetZoneBaselineXAxisOffset() const;
  double GetAngleVelConst() const;


  void SetExpZoneMarker(visualization_msgs::Marker marker);
  void SetDangerZoneMarker(visualization_msgs::Marker marker);
  void SetExpPolygonPoints(std::vector<Point> exp_polygon_points);
  void SetDangerPolygonPoints(std::vector<Point> danger_polygon_points);
  void SetFootprintVertices(geometry_msgs::PolygonStamped footprint_vertices);
  void SetZoneBaselineXAxisOffset(double value);

  void ClearDangerZoneMarker();
  void ClearExpZoneMarker();
  void ClearDangerPolygonPoints();
  void ClearExpPolygonPoints();


private:
  /* Laser */
  LaserInfo laser_;
  std::string laser_name_;
  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;

  std::vector<Point> danger_polygon_points_;
  std::vector<Point> exp_polygon_points_;

  visualization_msgs::Marker danger_zone_marker_;
  visualization_msgs::Marker exp_zone_marker_;

  geometry_msgs::PolygonStamped footprint_vertices_;
  Footprint footprint_;

  double zone_baseline_x_axis_offset_;
  double angle_vel_const_;
  double safety_dist_mult_;
  double max_accel_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_SAFETY_ZONE_SM_SAFETYZONE_BASE_H

