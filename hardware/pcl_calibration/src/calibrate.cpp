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

// TF stuff
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

// Math
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Laser
#include <laser_geometry/laser_geometry.h>

// TinyXML
#include <tinyxml.h>

// Point cloud publishers
ros::Publisher floor_pub;
ros::Publisher box_pub;
ros::Publisher laser_pub;
ros::Publisher box_line_pub;
ros::Publisher calibrated_pub;
tf::TransformListener *listener;
laser_geometry::LaserProjection projector;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr laser_cloud(new PointCloud);

// Icky global data for the laser state
float laser_angle;
Eigen::Vector3f laser_mid;
bool found_laser = false;
float floor_offset = 0;
std::string camera;
std::string camera_frame;
std::string camera_cloud;
std::string extrinsics_path;
TiXmlDocument xml_doc;  // The calibration document
/**
 * Get an affine transform object between two TF frames
 * @param  targetFrame The target TF
 * @param  sourceFrame The source TF
 * @return             An Affine transform between the frames
 */
Eigen::Affine3f getAffineFromTF(const std::string targetFrame, const std::string sourceFrame)
{
  Eigen::Affine3d affine_d;  // tf_eigen requires doubles for some reason
  tf::StampedTransform tf_transform;
  listener->lookupTransform(targetFrame, sourceFrame, ros::Time::now(), tf_transform);
  tf::transformTFToEigen(tf_transform, affine_d);
  return affine_d.cast<float>();  // Cast to float since that's what the rest is working in
}
/**
 * Callback for the laser scan message
 * @param scan_in The laser scan message
 */
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if (!listener->waitForTransform(
        scan_in->header.frame_id,
        "laser_front_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
  {
     return;
  }

  // Get a point cloud from the laser scan
  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("laser_front_link", *scan_in, cloud, *listener);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);

  // Limit to 1.2-3 m in x plane (forward)
  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(laser_cloud);
  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(0.2, 2);
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
  seg.setDistanceThreshold(0.015);
  seg.setInputCloud(laser_cloud);
  seg.segment(*inliers, *coefficients);

  // Replace laser cloud with just the box points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(laser_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*laser_cloud);

  // Publish for debugging
  laser_pub.publish(laser_cloud);

  // Find the line equation params
  Eigen::Vector3f line_unit_vector(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
  Eigen::Vector3f point_on_line(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

  // Calculate the midpoint
  Eigen::Vector4f centroid_mid;
  pcl::compute3DCentroid(*laser_cloud, centroid_mid);

  // Store the midpoint and the angle of the box for the cloud callback to use
  laser_mid = Eigen::Vector3f(centroid_mid.x(), centroid_mid.y(), 0);
  laser_angle = atan2(line_unit_vector.y(), line_unit_vector.x());
  found_laser = true;
}
/**
 * Update a property in the extrinsics.urdf document
 * @param name  The property's name
 * @param value The property's value
 */
void update_property(const std::string name, const float& value)
{
  std::stringstream full_name;
  full_name << name << camera;

  // full_name.str()
  TiXmlElement* property = xml_doc.FirstChildElement("robot")->FirstChildElement("property")->ToElement();
  while (property)
  {
    if (full_name.str() == property->Attribute("name"))
    {
      // ROS_INFO_STREAM("updating: " << property->Attribute("name") << " = " <<value);
      property->SetDoubleAttribute("value", value);
    }
    property = property->NextSiblingElement("property");
  }
}
/**
 * Callback to handle point cloud messages
 * @param cloud The point cloud message
 */
void cloud_cb(const PointCloud::ConstPtr& cloud)
{
  // Wait until the laser data is ready
  if (!found_laser)  return;

  // Delcare some clouds
  PointCloud::Ptr cloud_filtered(new PointCloud);
  PointCloud::Ptr floor_cloud(new PointCloud);
  PointCloud::Ptr box_cloud(new PointCloud);
  PointCloud::Ptr box_line_cloud(new PointCloud);
  PointCloud::Ptr plane_calibrated(new PointCloud);
  pcl_ros::transformPointCloud(camera_frame, *cloud, *cloud_filtered, *listener);

  // Through away points that are closer than 1.2m and further than 3m
  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(cloud_filtered);
  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(1.2, 3);
  pass_through.filter(*cloud_filtered);

  // Remove outliers from the input cloud
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);
  sor.filter(*cloud_filtered);

  // RANSAC plane finding pass #1: Find the floor plane
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  // Extract the floor cloud from the filtered cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*floor_cloud);

  // Extract everything but the floor cloud from the filtered cloud
  extract.setNegative(true);
  extract.filter(*cloud_filtered);

  // RANSAC take #2: Find the box
  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers2, *coefficients2);

  // Extract the box from the filtered cloud
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers2);
  extract.setNegative(false);
  extract.filter(*box_cloud);

  // ground normal
  Eigen::Vector3f norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  float height = coefficients->values[3] / norm.norm();

  // Publish the floor
  floor_pub.publish(floor_cloud);
  Eigen::Vector3f groundNorm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  Eigen::Vector3f boxNorm(coefficients2->values[0], coefficients2->values[1], coefficients2->values[2]);
  Eigen::Affine3f groundToCamera(Eigen::Affine3f::Identity());
  groundToCamera.rotate(Eigen::Quaternionf().setFromTwoVectors(groundNorm, Eigen::Vector3f(0, 0, 1)));
  groundToCamera.translate(Eigen::Vector3f(0.0, 0.0, height - floor_offset));
  ROS_INFO_STREAM("Height: " << (height-floor_offset));

  // Get 'native' camera space
  pcl_ros::transformPointCloud(camera_frame, *box_cloud, *box_cloud, *listener);

  // Use newly created ground to camera to move to base_link space
  pcl::transformPointCloud(*box_cloud, *box_cloud, groundToCamera);
  box_cloud->header.frame_id = "base_link";
  box_pub.publish(box_cloud);  // "base link" version of box

  // Convert to base frame
  Eigen::Vector3f boxNormBase;
  boxNormBase = groundToCamera * boxNorm;

  // Get line of box at hokoyu level
  pcl_ros::transformPointCloud("laser_front_link", *box_cloud, *box_line_cloud, *listener);
  pass_through.setInputCloud(box_line_cloud);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(-0.005, 0.005);  // 25cm up or so
  pass_through.filter(*box_line_cloud);

  // Calculate midpoint and angle in laser space
  float box_angle;
  Eigen::Vector3f mid;
  Eigen::Vector4f centroid_mid;
  pcl::compute3DCentroid(*box_line_cloud, centroid_mid);
  mid = Eigen::Vector3f(centroid_mid.x(), centroid_mid.y(), 0);
  box_angle = M_PI/2 + atan2(boxNormBase.y(), boxNormBase.x());  // rot. 90 deg. because it's a perpendicular normal

  // Force the laser and the box angle into the same 180 degree arc
  if (laser_angle < 0)  laser_angle += M_PI;
  if (box_angle < 0)  box_angle+= M_PI;

  // Calculate the angle difference
  float angle_diff = laser_angle - box_angle;
  ROS_INFO_STREAM("Laser angle: " << pcl::rad2deg(laser_angle) << " vrs " << pcl::rad2deg(box_angle)
    << " diff: " << pcl::rad2deg(angle_diff));

  // Rotate the midpoint by the angle difference
  Eigen::Vector3f mid_rotated = Eigen::AngleAxisf(angle_diff, Eigen::Vector3f::UnitZ()) * mid;

  // The find the translation between the two line pointclouds
  Eigen::Vector3f translation = laser_mid - mid_rotated;
  ROS_INFO_STREAM("Found angles and midpoints: angle " << pcl::rad2deg(angle_diff) << " translation: "
    << translation.x() << ", " << translation.y());

  // Create an affine transform to rotate and translate based on the laser deltas
  Eigen::Affine3f calibration = Eigen::Affine3f::Identity();
  calibration.translation() << translation.x(), translation.y(), 0.0;
  calibration.rotate(Eigen::AngleAxisf(angle_diff, Eigen::Vector3f::UnitZ()));

  // Publish the matched box-line point cloud
  pcl::transformPointCloud(*box_line_cloud, *box_line_cloud, calibration);
  box_line_pub.publish(box_line_cloud);

  // ==== Output calibrated cloud ====

  // Show the calibrated base link point cloud. Get a fresh copy of the input cloud
  pcl_ros::transformPointCloud(camera_frame, *cloud, *plane_calibrated, *listener);

  // Apply the hokoyu offset/yaw matrix in hokuyuo space
  groundToCamera = getAffineFromTF("laser_front_link", "base_link") * groundToCamera;
  groundToCamera = calibration * groundToCamera;
  groundToCamera = getAffineFromTF("base_link", "laser_front_link") * groundToCamera;

  // Use newly created ground to camera to move to base_link space without TF!
  pcl::transformPointCloud(*plane_calibrated, *plane_calibrated, groundToCamera);
  plane_calibrated->header.frame_id = "base_link";  // Override the frame ID since we've set it manually
  calibrated_pub.publish(plane_calibrated);  // "base link" version of box

  // ==== Spit out the final transform ====
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(groundToCamera, x, y, z, roll, pitch, yaw);
  ROS_INFO("    xyz: [%f, %f, %f]\n    rpy: [%.10f, %.10f, %.10f]\n", x, y, z, roll, pitch, yaw);
  // update_property("cam_px_", x);
  // update_property("cam_py_", y);
  // update_property("cam_pz_", z);
  // update_property("cam_or_", roll);
  // update_property("cam_op_", pitch);
  // update_property("cam_oy_", yaw);
}
/**
 * Sig-int handler(ctrl-c on the command line)
 * @param param The signal number
 */
void siginthandler(int param)
{
  // ROS_INFO("User pressed Ctrl+C, writing XML...");
  // xml_doc.SaveFile(extrinsics_path);
  exit(1);
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
  ros::init(argc, argv, "pcl_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle nh2("~");
  nh2.param<std::string>("camera", camera, "right");
  std::stringstream fmt;
  fmt << "camera_" << camera << "_rgb_frame";
  camera_frame = fmt.str();
  fmt.clear();
  fmt.str("");
  fmt << "/camera_" << camera << "/depth/points";
  camera_cloud = fmt.str();
  ROS_INFO_STREAM("Calibrating " << camera_frame << " using " << camera_cloud);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_kinect = nh.subscribe<PointCloud>(camera_cloud, 1, cloud_cb);
  ros::Subscriber sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_cb);

  // Get a TF listener object
  listener = new tf::TransformListener;
  listener->waitForTransform(camera_frame, "base_link", ros::Time::now(), ros::Duration(1.));
  listener->waitForTransform("base_footprint", "base_link", ros::Time::now(), ros::Duration(1.));
  tf::StampedTransform tf_transform;
  listener->lookupTransform("base_footprint", "base_link", ros::Time::now(), tf_transform);
  floor_offset = static_cast<float>(tf_transform.getOrigin().z());
  ROS_INFO_STREAM("Floor offset from URDF: " << floor_offset);

  // Create a ROS publisher for the output point cloud
  floor_pub = nh.advertise<PointCloud> ("pcl_calibration/floor", 1);
  box_pub = nh.advertise<PointCloud> ("pcl_calibration/box", 1);
  laser_pub = nh.advertise<PointCloud> ("pcl_calibration/laser", 1);
  box_line_pub = nh.advertise<PointCloud> ("pcl_calibration/box_line", 1);
  calibrated_pub = nh.advertise<PointCloud> ("pcl_calibration/cloud", 1);

  // Load extrinsics
  /*nh2.param<std::string>("file", extrinsics_path,
    "/home/avidbot/Dev/avidbots/ros_hydro/src/avidbots_description/shiny/urdf/extrinsics.urdf.xacro");
  ROS_INFO_STREAM("Updating file: " << extrinsics_path);
  xml_doc = TiXmlDocument(extrinsics_path);
  if (!xml_doc.LoadFile() )
  {
    ROS_INFO_STREAM("Couildn't load " << extrinsics_path);
  }
  else
  {
    ROS_INFO_STREAM("Loaded " << extrinsics_path);
    signal(SIGINT, siginthandler);
  }*/

  // Spin
  ros::spin();
}
