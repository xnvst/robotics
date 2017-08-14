/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name  safetyzone_messenger
 * @brief Header file for SafetyZoneMessenger class
 * @author Keshav Iyengar
 */

#ifndef AVIDBOTS_SAFETY_MONITOR_MESSENGERS_SAFETYZONE_MESSENGER_H
#define AVIDBOTS_SAFETY_MONITOR_MESSENGERS_SAFETYZONE_MESSENGER_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

// LOCAL
#include <avidbots_library/safety_zone/properties.h>
#include <avidbots_library/geometry/geometry_msgs/geometry_util.h>
#include <avidbots_library/geometry/geometry_msgs/polygon.h>
#include <avidbots_msgs/safety_check.h>

// TF
#include <tf/transform_listener.h>

// Laser
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>


// Safety Zone
#include "avidbots_safety_monitor/safety_zone/sm_safetyzone_base.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class ROSInterface;

class SafetyZoneMessenger
{
public:
  explicit SafetyZoneMessenger(ROSInterface *ros);
  ~SafetyZoneMessenger();

  /* Action Functions */
  void Initialize();
  void PublishMCUOutputCommand();
  void PublishDangerZoneObstacle(bool status);

  /* Helper Functions */
  void GetParamSettings();
  void InitializePublishersAndSubscribers();
  void InitializeTimers();
  bool WaitForLaserTransform(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
  // Callbacks
  void SafetyZoneTimerCallBack(const ros::TimerEvent&);
  void EnableZonesCallBack(const std_msgs::Bool& enable_zones_msg);
  void LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
  void TwistCmdCallBack(const geometry_msgs::TwistPtr& vel_msg);
  void LocalControllerSafetyCheckCallBack(const avidbots_msgs::safety_check& safety_check_msg);

  PointCloud::Ptr GetLaserPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
  void RetractSafetyZone(Point& velocity_vector, double val);

  // Getters
  ROSInterface*   ros_;
  ros::NodeHandle node_;

  SMSafetyZoneBase* danger_zone_;
  SMSafetyZoneBase* exp_safetyzone_;

  // Publishers
  ros::Publisher laser_pub_,
                 danger_obstacle_pub_,
                 danger_vis_pub_,
                 manual_override_cmd_publisher_,
                 enable_zones_pub_,
                 mcu_output_pub_,
                 vel_vis_pub_;

  // Subscribers
  ros::Subscriber enable_zones_sub_,
                  laser_sub_,
                  twist_cmd_sub_,
                  local_controller_safety_check_sub_;

  // Timers
  ros::Timer safetyzone_timer_;

  bool enable_safety_zone_monitor_,
       enable_danger_zone_,
       enable_dangerzone_zero_vel_,
       skip_danger_obstacle_reset_,
       enable_exp_safety_zone_;

  double obstacle_timeout_;

  Point velocity_vector_;
  double ang_vel_;

  bool has_laser_tf_;

  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;
  tf::StampedTransform transform_;

  LaserInfo laser_;
  std::string laser_name_;
  geometry_msgs::PolygonStamped footprint_vertices_;
  double zone_baseline_x_axis_offset_;

  ErrorStatus danger_zone_status_;
  ErrorStatus exp_safetyzone_status_;
  ErrorStatus local_controller_safety_status_;
};

#endif  // AVIDBOTS_SAFETY_MONITOR_MESSENGERS_SAFETYZONE_MESSENGER_H
