/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	avidbots_sensor_safety.h
 * @brief	Header file containing the Avidbots Sensor Safety class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_H
#define AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_H

// ROS
#include <ros/ros.h>
#include <ecl/time.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

// CPP
#include <string>
#include <yaml-cpp/yaml.h>

// BOOST
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/functional/hash.hpp>

// LOCAL
#include <avidbots_library/socket_can/avidbots_can_open_constants.h>
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/diagnostics_states.h>
#include <avidbots_msgs/diagnostics.h>
#include "avidbots_sensor_safety/avidbots_sensor_safety_constants.h"
#include <avidbots_library/get_param/get_param_util.h>
#include <avidbots_msgs/back_wheel_encoder_status.h>
#include <avidbots_msgs/sensor_toggle.h>
#include <avidbots_msgs/sensor_safety_status.h>
#include <avidbots_msgs/mcu_status.h>

class AvidbotsSensorSafety
{
public:
  AvidbotsSensorSafety();
  ~AvidbotsSensorSafety();

  void Init(const ros::NodeHandle& private_nh,
                  ros::NodeHandle* node_handle);
  void PublishSensorSafetyUpdate();
  void PublishSensorSafetyStatus();

  void GetParamSettings(const ros::NodeHandle& private_nh);
  void ManagePubAndSub();

private:
  // ROS publisher/subscriber
  ros::NodeHandle*  node_handle_;

  ros::Subscriber   lasers_status_sub_;
  ros::Subscriber   laser_front_sub_;
  ros::Subscriber   laser_back_sub_;
  ros::Subscriber   odometry_status_sub_;
  ros::Subscriber   encoder_status_sub_;
  ros::Subscriber   camera_left_status_sub_;
  ros::Subscriber   camera_right_status_sub_;
  ros::Subscriber   sensor_toggle_sub_;
  ros::Subscriber   mcu_status_sub_;

  ros::Publisher    sensor_safety_update_pub_;
  ros::Publisher    sensor_safety_status_pub_;

  int lasers_status_;
  int laser_front_status_;
  int laser_back_status_;
  int odometry_status_;
  int encoder_status_;
  int camera_left_status_;
  int camera_right_status_;

  int old_lasers_status_;
  int old_laser_front_status_;
  int old_laser_back_status_;
  int old_odometry_status_;
  int old_encoder_status_;
  int old_camera_left_status_;
  int old_camera_right_status_;

  int mcu_state_;

  bool front_laser_;
  bool back_laser_;
  int laser_number_;

  /* Sensor Enables (True = Check sensor readings, false = Ignore sensor readings */
  bool global_enable_;
  bool lasers_enable_;
  bool odometry_enable_;
  bool encoder_enable_;
  bool camera_left_enable_;
  bool camera_right_enable_;

  double lasers_timeout_;
  double odometry_timeout_;
  double encoder_timeout_;
  double camera_timeout_;

  // Timers
  ros::Timer main_timer_;

  ecl::StopWatch lasers_stopwatch_;
  ecl::StopWatch laser_front_stopwatch_;
  ecl::StopWatch laser_back_stopwatch_;
  ecl::StopWatch odometry_stopwatch_;
  ecl::StopWatch encoder_stopwatch_;
  ecl::StopWatch camera_left_stopwatch_;
  ecl::StopWatch camera_right_stopwatch_;

  void ResetSensorSafetyStatus();
  void MainTimerCallBack(const ros::TimerEvent &);

  // CallBacks
  void LasersStatusCallback(const sensor_msgs::LaserScanPtr &LaserScan);

  void FrontLaserStatusCallback(const sensor_msgs::LaserScanPtr &LaserScan);
  void BackLaserStatusCallback(const sensor_msgs::LaserScanPtr &LaserScan);

  void OdometryStatusCallback(const nav_msgs::OdometryPtr &Odometry);
  void EncoderStatusCallback(const avidbots_msgs::back_wheel_encoder_statusPtr &back_wheel_encoder_status);
  void CameraLeftStatusCallback(const avidbots_msgs::diagnostics_statesPtr &diagnostics_states);
  void CameraRightStatusCallback(const avidbots_msgs::diagnostics_statesPtr &diagnostics_states);
  void SensorToggleCallback(const avidbots_msgs::sensor_toggle &sensor_toggle);  // @Paul
  void MCUStatusCallBack(const avidbots_msgs::mcu_status& mcu_status_msg);
};

#endif  // AVIDBOTS_SENSOR_SAFETY_AVIDBOTS_SENSOR_SAFETY_H
