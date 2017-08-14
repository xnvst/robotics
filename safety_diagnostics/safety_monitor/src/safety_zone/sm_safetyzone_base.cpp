/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    safetyzone_base
 * @brief   Implementation file for SMSafetyZoneBase class
 * @author  Keshav Iyengar
 */

// CPP
#include <string>
#include <vector>
#include <algorithm>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/safety_zone/sm_safetyzone_base.h"
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include <avidbots_library/get_param/get_param_util.h>


/**
 * @name        SMSafetyZoneBase
 * @brief       Constructor.
 */
SMSafetyZoneBase::SMSafetyZoneBase()
{}

/**
 * @name        GetParam
 * @brief       Get parameters from parameter server.
 */
void SMSafetyZoneBase::GetParam()
{
  std::string motion_model;
  GetParamUtil::GetParam("/robot_properties/motion_model", motion_model, "bicycle");

  GetParamUtil::GetFootprintParam("/footprint", footprint_vertices_, GetParamUtil::GetDefaultFootprintPolygon());
  SetFootprintVertices(footprint_vertices_);
  geometry_msgs::util::FootprintProperties footprint_properties(footprint_vertices_);
  footprint_.positive_max_x_position = footprint_properties.max_x;
  GetParamUtil::GetParam("/robot_properties/footprint_length", footprint_.length, 1.064);
  GetParamUtil::GetParam("/robot_properties/footprint_width", footprint_.width, .78);

  GetParamUtil::GetParam("/sm_properties/zone_baseline_x_axis_offset", zone_baseline_x_axis_offset_, 0.0);
  SetZoneBaselineXAxisOffset(zone_baseline_x_axis_offset_);
  GetParamUtil::GetParam("/sm_properties/angular_velocity_constant", angle_vel_const_, 0.5);
  GetParamUtil::GetParam("/sm_properties/safety_distance_multiplier", safety_dist_mult_, 1);
  double wheel_radius;
  double gear_ratio;
  double max_rpm_speed;

  if (motion_model == "bicycle")
  {
    GetParamUtil::GetParam("/robot_properties/front_wheel_radius", wheel_radius, 0.1);
    GetParamUtil::GetParam("/robot_properties/front_wheel_gear_ratio", gear_ratio, 27.271);
  }
  else
  {
    GetParamUtil::GetParam("/robot_properties/wheel_radius", wheel_radius, 0.1016);
    GetParamUtil::GetParam("/robot_properties/gear_ratio", gear_ratio, 44.4);
  }
  if (!node_.getParam("/robot_properties/laser_link", laser_name_))
  {
    ROS_ERROR("Couldn't load param from `/robot_properties/laser_link");
  }
  GetParamUtil::GetParam("/robot_properties/motor_controller_max_accel", max_rpm_speed, 3000);

  /* Calculate the max acceleration (convert from rpm to m/s^2)
   * 1) RPM/s to rad/s^2
   * 2) rad/s^2 (motor) to rad/s^2 (wheel)
   * 3) rad/s^2 (wheel) to m/s^2
   */
  double kRPMPerSecToRadPerSecSquared = 2 * M_PI / 60;
  double motor_accel = max_rpm_speed * kRPMPerSecToRadPerSecSquared / gear_ratio;
  max_accel_ = motor_accel * wheel_radius;

  ROS_INFO_STREAM("Motor accel: " << motor_accel << " Max accel: " << max_accel_);
}

/**
 * @name    InitLaserInfo()
 * @brief   Initialization of laser properties
 */
void SMSafetyZoneBase::InitLaserInfo()
{
  /* Listen to tf server for laser rotation and displacement from base link */
  tf::TransformListener listener_;
  tf::StampedTransform transform;
  bool locking_flag = true;

  listener_.waitForTransform("base_footprint", "base_link", ros::Time::now(), ros::Duration(1.0));
  /* Locking call until tf is recieved */
  while (locking_flag)
  {
    try
    {
      listener_.waitForTransform("/base_link", laser_name_,
                                ros::Time(0), ros::Duration(1000));
      ROS_WARN("Waiting for transform /base_link to laser");
      listener_.lookupTransform("/base_link", laser_name_,
                               ros::Time(0), transform);
      locking_flag = false;
    }
    catch (tf::TransformException ex)
    {
      locking_flag = true;
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  /* This is info about where the laser is located. */
  laser_.rotation = tf::getYaw(transform.getRotation());
  laser_.delta_forward = transform.getOrigin().x();
  laser_.delta_side = transform.getOrigin().y();
  laser_.delta_phi = atan2(laser_.delta_side, laser_.delta_forward);
  laser_.delta_magnitude =
      geometry_msgs::util::Norm2(laser_.delta_forward, laser_.delta_side);
}


/**
 * @name    StoppingDistance()
 * @brief   Returns the maximum stopping distance
 * param[in] vector: Initial velocity vector
 * param[in] accel: Acceleration
 * param[in] multiplier: Factor to increase stopping distance. Default to 1 if none provided
 */
double SMSafetyZoneBase::StoppingDistance(Point vector, double accel, double multiplier)
{
  double vel_mag = geometry_msgs::util::Norm2(vector.x, vector.y);
  return vel_mag * vel_mag / (2 * accel) * multiplier;
}


/**
 * @name  CheckSafetyZonePointCloud
 * @brief Creates a point cloud from the laser points with
 *        respect to the base link. Then calls the CreateSafetyZone
 *        method and PinPolygon method to create and check the safetyzone.
 * @param[in] scan: The values of the scan from laser. Organized
 *                  with index as angle and value as range.
 * @param[in] safety_zone_check: True if there are no points in the safety zone
 * @param[in] polygon_points: True if there are no points in the safety zone
 */
bool SMSafetyZoneBase::CheckZonePointCloud(
  PointCloud::Ptr laser_cloud,
  bool safety_zone_check,
  std::vector<Point> polygon_points)
{
  /* Itereate through every single point cloud point */
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = laser_cloud->begin(); it != laser_cloud->end(); it++)
  {
    Point laser_point;
    laser_point.x = it->x;
    laser_point.y = it->y;

    bool last_safety_zone_check = safety_zone_check;
    /* Check if point is in the Safetyzone */
    safety_zone_check = PinPolygon(laser_point,
                                   polygon_points,
                                   last_safety_zone_check);
    if (!safety_zone_check)
    {
      return false;
    }
  }
  return true;
}


/**
 * @name    PinPolygon()
 * @brief   Function that checks if a given point is in the safetyzone.
 * @param   [in] point: The cartesian coordinates of the point to check.
 */
bool SMSafetyZoneBase::PinPolygon(Point &point, std::vector<Point> polygon_points,
                                  bool &safety_zone)
{
  Point new_point;
  new_point.x = point.x;
  new_point.y = point.y;

  /* in_zone_val is what the cross product is checked aganist. If there are no points in the zone (safety_zone == true)
   * set it to 0. If there is a point, set it a little outside the zone to avoid flickering. */
  double in_zone_val = 0;
  if (!safety_zone)
  {
    in_zone_val = 0.1;
  }

  bool result = false;
  int num_vert = polygon_points.size() - 1;

  for (int i = 0; i < num_vert; i++)
  {
    Point vec_a;
    Point vec_b;
    vec_a = polygon_points.at(i);
    vec_b = polygon_points.at((i+1) % num_vert);

    Point polygon_edge = Polygon::CreateVector(vec_b, vec_a);
    Point laser_point = Polygon::CreateVector(new_point, vec_a);

    double laser_point_dist = Polygon::XProduct(polygon_edge, laser_point);
    if (laser_point_dist < in_zone_val && !result)
    {
      result = false;
    }
    else
    {
      result = true;
      return result;
    }
  }
  return result;
}

/**
 * @name    XProductCheckSide()
 * @brief   Function that does the velocity expansion. Creates a vector normal to the velocity
 *          vector and does a cross product with each vertex. We can determine which vertices to expand
 *          based on xproduct (side towards velocity or away).
 *          If the angle of the normal vector is -90 or 0, simply add the velocity vector to the correct vertices.
 *          Otherwise, we need to only add the x-value to the side corner vertices and add normally to the front corner
 *          vertices.
 * @param   [in] vector: The velocity vector of the robot.
 * @param   [in] polygon_points: The polygon points of the safety zone
 */
std::vector<Point> SMSafetyZoneBase::XProductCheckSide(Point &vector, std::vector<Point> polygon_points)
{
  /* Normal vector of the velocity vector */
  Point n_velocity_vector;
  n_velocity_vector.x = -vector.y;
  n_velocity_vector.y = vector.x;

  double stop_dist = StoppingDistance(vector, max_accel_, safety_dist_mult_);

  int num_vert = polygon_points.size();

  for (int i = 0; i < num_vert; i++)
  {
    double temp = Polygon::XProduct(n_velocity_vector, polygon_points.at(i));

    /* Vertex is on the velocity vector side */
    if (temp < 0)
    {
      double vector_ang = atan2(vector.y, vector.x);  // vector angle is in radians

      if (vector.x == 0 || vector.y == 0)
      {
        if (vector.x != 0)
        {
          polygon_points.at(i).x += vector.x/fabs(vector.x) * fabs(stop_dist * cos(vector_ang));
        }
        if (vector.y != 0)
        {
          polygon_points.at(i).y += vector.y/fabs(vector.y) * fabs(stop_dist * sin(vector_ang));
        }
      }
      /* If i is either of the base line vertices (they remain on the x-axis at all times) */
      else if (i == 0 || i == 1)
      {
        if (vector.y != 0)
        {
          polygon_points.at(i).y += vector.y/fabs(vector.y) * fabs(stop_dist * sin(vector_ang));
        }
      }
      /* The rest of the vertices are free to move but cannot go below the base line vertices */
      else
      {
        if (vector.x != 0)
        {
          polygon_points.at(i).x += vector.x/fabs(vector.x) * fabs(stop_dist * cos(vector_ang));
        }
        if (vector.y != 0)
        {
          polygon_points.at(i).y += vector.y/fabs(vector.y) * fabs(stop_dist * sin(vector_ang));
        }

        /* Set vertex to original polygon point if the vertex goes past baseline vertices. */
        if (polygon_points.at(i).x < std::min(polygon_points.at(0).x, polygon_points.at(1).x))
        {
          polygon_points.at(i).x = footprint_.positive_max_x_position;
        }
      }
    }
  }
  return polygon_points;
}

///////////////////////
/* HELPER FUNCTIONS */
/////////////////////

/**
 * @name    DrawPolygon()
 * @brief   Pushes final points to be published in rviz.
 * @param   [in] polygon_points: List of points of a polygon.
 * @param   [in] zone_marker: Marker used to draw points in rviz
 */
void SMSafetyZoneBase::DrawPolygon(std::vector<Point> &polygon_points,
                                            visualization_msgs::Marker &zone_marker)
{
  int num_vert = polygon_points.size();
  geometry_msgs::Point points;
  for (int i = 0; i < num_vert; i++)
  {
    points.x = polygon_points.at(i).x;
    points.y = polygon_points.at(i).y;
    zone_marker.points.push_back(points);
    points.x = polygon_points.at((i+1) % num_vert).x;
    points.y = polygon_points.at((i+1) % num_vert).y;
    zone_marker.points.push_back(points);
  }
}


/**
 * @name    CreatePolygon()
 * @brief   Returns vector of points of the initial polygon formed
 * param[in] footprint_vertices: Footprint containing vector of vertex points
 * param[in] baseline_x_axis_offset: The baseline x axis offset
 * param     polygon_points: The resulting vector of points of the initial polygon formed   
 */
std::vector<Point> SMSafetyZoneBase::CreatePolygon(
  geometry_msgs::PolygonStamped footprint_vertices, double baseline_x_axis_offset)
{
  int size = footprint_vertices.polygon.points.size();
  std::vector<Point> polygon_points;
  Point origin;
  origin.x = 0;
  origin.y = 0;
  polygon_points.resize(size, origin);

  for (int i = 0; i < size; i++)
  {
    /* Initialize baseline vertices */
    if (i == 0 || i == 1)
      polygon_points.at(i).x = baseline_x_axis_offset;
    else
      polygon_points.at(i).x = footprint_vertices.polygon.points[i].x;

    polygon_points.at(i).y = footprint_vertices.polygon.points[i].y;
  }
  return polygon_points;
}

//////////////
/* Setters */
/////////////

/**
 * @name    SetExpPolygonPoints
 * @brief   Set the exp safety zone polygon points
 * @param   exp_polygon_points: Points to set
 */
void SMSafetyZoneBase::SetExpPolygonPoints(std::vector<Point> exp_polygon_points)
{
  exp_polygon_points_ = exp_polygon_points;
}

/**
 * @name    SetDangerPolygonPoints
 * @brief   Set the exp safety zone polygon points
 * @param   danger_polygon_points: Points to set
 */
void SMSafetyZoneBase::SetDangerPolygonPoints(std::vector<Point> danger_polygon_points)
{
  danger_polygon_points_ = danger_polygon_points;
}

/**
 * @name    SetExpZoneMarker
 * @brief   Set the exp safety zone markers
 * @param   marker: markers for visualization
 */
void SMSafetyZoneBase::SetExpZoneMarker(visualization_msgs::Marker marker)
{
  exp_zone_marker_ = marker;
}

/**
 * @name    SetDangerZoneMarker
 * @brief   Set the danger safety zone markers
 * @param   marker: markers for visualization
 */
void SMSafetyZoneBase::SetDangerZoneMarker(visualization_msgs::Marker marker)
{
  danger_zone_marker_ = marker;
}

/**
 * @name    SetFootprintVerticies
 * @brief   Set the footprint vertices
 * @param   footprint_vertices: footprint to set
 */
void SMSafetyZoneBase::SetFootprintVertices(geometry_msgs::PolygonStamped footprint_vertices)
{
  footprint_vertices_ = footprint_vertices;
}

/**
 * @name    SetZoneBaselineXAxisOffset
 * @brief   set the baseline x axis offset
 * @param   value: Set the value
 */
void SMSafetyZoneBase::SetZoneBaselineXAxisOffset(double value)
{
  zone_baseline_x_axis_offset_ = value;
}

//////////////
/* Getters */
/////////////

/**
 * @name    GetExpZoneMarker
 * @brief   Get the exp zone marker
 */
visualization_msgs::Marker SMSafetyZoneBase::GetExpZoneMarker() const
{
  return exp_zone_marker_;
}

/**
 * @name    GetDangerZoneMarker
 * @brief   Get the danger zone marker
 */
visualization_msgs::Marker SMSafetyZoneBase::GetDangerZoneMarker() const
{
  return danger_zone_marker_;
}

/**
 * @name    GetExpPolygonPoints
 * @brief   Get the Exp polygon points
 */
std::vector<Point> SMSafetyZoneBase::GetExpPolygonPoints() const
{
  return exp_polygon_points_;
}

/**
 * @name    GetDangerPolygonPoints
 * @brief   Get the danger polygon points
 */
std::vector<Point> SMSafetyZoneBase::GetDangerPolygonPoints() const
{
  return danger_polygon_points_;
}

/**
 * @name    GetFootprintVertices
 * @brief   Get the footprint vertices
 */
geometry_msgs::PolygonStamped SMSafetyZoneBase::GetFootprintVertices() const
{
  return footprint_vertices_;
}

/**
 * @name    GetBaselineXAxisOffset
 * @brief   Get the zone baseline x axis offset
 */
double SMSafetyZoneBase:: GetZoneBaselineXAxisOffset() const
{
  return zone_baseline_x_axis_offset_;
}

/**
 * @name    GetGetAngleVelConst
 * @brief   Get the angle vel
 */
double SMSafetyZoneBase:: GetAngleVelConst() const
{
  return angle_vel_const_;
}

////////////////////
/* Clear Function */
////////////////////

/**
 * @name    ClearDangerZoneMarker
 * @brief   Clear the danger zone markers
 */
void SMSafetyZoneBase::ClearDangerZoneMarker()
{
  danger_zone_marker_.points.clear();
}

/**
 * @name    ClearExpZoneMarker
 * @brief   Clear the exp zone markers
 */
void SMSafetyZoneBase::ClearExpZoneMarker()
{
  exp_zone_marker_.points.clear();
}

/**
 * @name    ClearDangerPolygonPoints
 * @brief   Clear the danger polygon points
 */
void SMSafetyZoneBase::ClearDangerPolygonPoints()
{
  danger_polygon_points_.clear();
}

/**
 * @name    ClearExpPolygonPoints
 * @brief   Clear the exp polygon points
 */
void SMSafetyZoneBase::ClearExpPolygonPoints()
{
  exp_polygon_points_.clear();
}
