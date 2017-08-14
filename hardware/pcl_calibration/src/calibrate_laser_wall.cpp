/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ , _\   ____
 *  \ \  __ \/\ \/\ \/\ \  /'_` \ \ '__`\  / __`\ \ \/  /', __\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___, _\ \_, __/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__, _ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 *
 * To use:
 * Stop the avidbots service then run:
 * roslaunch avidbots_launch kinect2x2.launch
 *
 * Put a box 1.2-3m in front of the left camera
 * rosrun pcl_calibration calibrate "_camera:=right"
 * ctrl-c to save/quit
 *
 * Move the box 1.2-3m in front of the left camera
 * rosrun pcl_calibration calibrate "_camera:=left"
 * ctrl-c to save/quit
 *
 * @name  calibrate.cpp
 * @brief  A program that automatically calibrates kinect position/rotation
 * @author Joseph DUchesne
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

// TF stuff
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

// Math
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Laser
#include <laser_geometry/laser_geometry.h>


// Point cloud publishers
ros::Publisher laser_pub;
tf::TransformListener *listener;
laser_geometry::LaserProjection projector;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr laser_cloud(new PointCloud);

// Icky global data for the laser state
float laser_angle;
Eigen::Vector3f laser_mid;

/**
 * Callback for the laser scan message
 * @param scan_in The laser scan message
 */
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if (!listener->waitForTransform(
        scan_in->header.frame_id,
        "hokuyo_ust_20lx_f_laser_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
  {
     return;
  }

  // Get a point cloud from the laser scan
  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("hokuyo_ust_20lx_f_laser_link", *scan_in, cloud, *listener);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);

  // Limit to 2.8, 3.3 m in x plane (forward)
  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(laser_cloud);
  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(0, 3.3);
  pass_through.filter(*laser_cloud);

  // Limit to -1 to +1 m in the y plane (side to side)
  pass_through.setFilterFieldName("y");
  pass_through.setFilterLimits(-2, 2);
  pass_through.filter(*laser_cloud);

  // Line finding RANSAC to get just the box, and its line equation
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(laser_cloud);
  seg.segment(*inliers, *coefficients);

  // Replace laser cloud with just the box points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(laser_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*laser_cloud);

  // Publish for debugging

  // Find the line equation params
  Eigen::Vector3f line_unit_vector(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
  Eigen::Vector3f point_on_line(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

  // Axes flipped from normal since we want degrees off of the vector (0,1) not (1,0) like normal
  laser_angle = atan(line_unit_vector.x()/line_unit_vector.y());

  laser_angle = laser_angle*180.0/M_PI;
  // Publish for debugging
  ROS_INFO_STREAM("Laser: " << line_unit_vector.x() << " " << line_unit_vector.y() << " " << laser_angle);
  laser_pub.publish(laser_cloud);
}

/**
 * Standard C program main
 * Start publishers, load data, listen for TF frames, and then ros spin
 *
 * @param  argc Argument count
 * @param  argv Argument value string array
 * @return      A status code
 */
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_laser_calibration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input laser scan
  ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_cb);

  // Get a TF listener object
  listener = new tf::TransformListener;
  listener->waitForTransform("base_footprint", "base_link", ros::Time::now(), ros::Duration(1.));

  // Create a ROS publisher for the output point cloud
  laser_pub = nh.advertise<PointCloud> ("pcl_calibration/laser", 1);

  ROS_INFO("Started");

  // Spin
  ros::spin();
}
