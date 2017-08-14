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
 * rosrun pcl_calibration calibrate_laser
 *
 * @name  calibrate_laser.cpp
 * @brief  A program that automatically calibrates the laser extents and angle
 * @author Joseph DUchesne
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

#include <avidbots_library/get_param/get_param_util.h>

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
PointCloud::Ptr right_cloud(new PointCloud);  // The cloud on the -ve y side of the robot
PointCloud::Ptr left_cloud(new PointCloud);  // The cloud on the +ve y side of the robot
std::string laser_link_name_;

double min_avg = 0;
double max_avg = 0;
double offset_avg = 0;
int readings = 0;
int attempts = 200;

/**
 * Callback for the laser scan message
 * @param scan_in The laser scan message
 */
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if (!listener->waitForTransform(
        scan_in->header.frame_id,
        laser_link_name_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
  {
     return;
  }

  // Get a point cloud from the laser scan
  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud(laser_link_name_, *scan_in, cloud, *listener);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);

  // Filter down to a 20cm x 20cm box around laser
  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(laser_cloud);

  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(-0.4, 0.4);
  pass_through.filter(*laser_cloud);

  pass_through.setFilterFieldName("y");
  pass_through.setFilterLimits(-0.4, 0.0);
  pass_through.filter(*right_cloud);

  pass_through.setFilterFieldName("y");
  pass_through.setFilterLimits(0.0, 0.4);
  pass_through.filter(*left_cloud);

  ROS_DEBUG("SIZES: %zd %zd %zd", laser_cloud->size(), right_cloud->size(), left_cloud->size());

  // Publish for debugging
  laser_pub.publish(laser_cloud);

  // Find the closest point on the right (-ve y) side of robot
  float right_angle = -4;
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = right_cloud->begin(); it != right_cloud->end(); it++)
  {
    float angle = atan2(it->y, it->x);
    if (angle > right_angle)
    {
      right_angle = angle;
    }
  }

  // Find the closest point on left (+ve y) side of robot
  float left_angle = 4;
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = left_cloud->begin(); it != left_cloud->end(); it++)
  {
    float angle = atan2(it->y, it->x);
    if (angle < left_angle)
    {
      left_angle = angle;
    }
  }

  attempts--;
  if (readings == 0 && attempts <= 0)
  {
    if (right_cloud->size() == 0 && left_cloud->size() == 0)
    {
      std::cout << "Calibration failed. Please adjust left and right calibration rods" << std::endl;
    }
    else if (right_cloud->size() == 0)
    {
      std::cout << "Calibration failed. Please adjust right calibration rod" << std::endl;
    }
    else if (left_cloud->size() > 0)
    {
      std::cout << "Calibration failed. Please adjust left calibration rod" << std::endl;
    }
    ros::shutdown();
  }

  /* Offset = yaw of laser in robot frame.  Used in calibration.yaml.
     +ve offset means laser rotated left (CCW), |right angle| > |left angle|
     -ve offset means laser rotated right (CW), |right angle| < |left angle| */
  float offset = -(right_angle + left_angle) / 2.0;
  if (right_cloud->size() > 0 && left_cloud->size() > 0 && offset < 0.1)
  {
    min_avg += right_angle;
    max_avg += left_angle;
    offset_avg += offset;
    readings++;

    std::cout << "Took reading " << readings << "/10" << std::endl;

    ROS_DEBUG_STREAM("Right_Min, Left_Max, Offset:  " << right_angle + 0.02
                    << "  " << left_angle - 0.02 << "  " << offset);
  }

  if (readings >= 10)
  {
    double readings_d = static_cast<double>(readings);
    min_avg /= readings_d;
    max_avg /= readings_d;
    offset_avg /= readings_d;

    double samples = (max_avg-min_avg)/scan_in->angle_increment;

    std::cout << "Calibration Value:" << std::endl;
    std::cout << "{\"scan_samples\": " << static_cast<int>(samples) << ", \"angle\":";
    std::cout << "{\"min\": " << (min_avg + 0.02) << ", \"max\": " << (max_avg - 0.02);
    std::cout << ", \"offset\": " << offset_avg << "}}" << std::endl;

    ros::shutdown();
  }
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
  std::cout << "Attempting laser calibration..." << std::endl;

  // Initialize ROS
  ros::init(argc, argv, "pcl_laser_calibration");
  ros::NodeHandle nh;

  GetParamUtil::GetParam("/robot_properties/laser_link", laser_link_name_, "laser_front_link");

  // Create a ROS subscriber for the input laser scan
  ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_cb);

  // Get a TF listener object
  listener = new tf::TransformListener;
  listener->waitForTransform("base_footprint", "base_link", ros::Time::now(), ros::Duration(1.));

  // Create a ROS publisher for the output point cloud
  laser_pub = nh.advertise<PointCloud> ("pcl_calibration/laser", 1);

  // Spin
  ros::spin();
}
