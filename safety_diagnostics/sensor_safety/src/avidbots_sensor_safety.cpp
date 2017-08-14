/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	avidbots_sensor_safety.cpp
 * @brief	Source File containing the AvidbotsSensorSafety class
 * @author	Feng Cao
 */

#include <string>
#include "avidbots_sensor_safety/avidbots_sensor_safety.h"



/**
 * @name 	AvidbotsSensorSafety
 * @brief	Default constructor
 */
AvidbotsSensorSafety::AvidbotsSensorSafety() :
global_enable_(false), front_laser_(0), back_laser_(0), laser_number_(0),
lasers_status_(SENSOR_OK), odometry_status_(SENSOR_OK), encoder_status_(SENSOR_OK),
camera_left_status_(SENSOR_OK), camera_right_status_(SENSOR_OK),
lasers_timeout_(FREQ_10HZ), odometry_timeout_(FREQ_15HZ),
encoder_timeout_(FREQ_15HZ), camera_timeout_(FREQ_10HZ)
{}

/**
 * @name  ~AvidbotsSensorSafety
 * @brief Destructor
 */
AvidbotsSensorSafety::~AvidbotsSensorSafety() {}

/**
 * @name        Init()
 * @brief       Initialize ros interface
 * @param[in]   private_nh, node handler
 * @param[in]   node_handle, node handler
 */
void AvidbotsSensorSafety::Init(const ros::NodeHandle& private_nh,
                                     ros::NodeHandle *node_handle)
{
  ROS_INFO_STREAM("AvidbotsSensorSafety Init");

  node_handle_        = node_handle;

  // Get parameters
  GetParamSettings(private_nh);

  // Initialize publishers and subscribers
  ManagePubAndSub();

  lasers_stopwatch_.restart();
  laser_front_stopwatch_.restart();
  laser_back_stopwatch_.restart();
  odometry_stopwatch_.restart();
  encoder_stopwatch_.restart();
  camera_left_stopwatch_.restart();
  camera_right_stopwatch_.restart();

  /* Create timer */
  main_timer_ = node_handle_->createTimer
      (ros::Duration(0.01), &AvidbotsSensorSafety::MainTimerCallBack, this);
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 * @param[in]   private_nh, node handler
 */
void AvidbotsSensorSafety::GetParamSettings
            (const ros::NodeHandle& private_nh)
{
  YAML::Node basenode = YAML::LoadFile("/etc/avidbots/calibration.yaml");
  string temp_front_ = "Initialize";
  string temp_back_ = "Initialize";
  if (basenode["laser-type"])
    if (basenode["laser-type"]["front"])
      temp_front_ = basenode["laser-type"]["front"].as<string>();
    if (basenode["laser-type"]["back"])
      temp_back_ = basenode["laser-type"]["back"].as<string>();

  ROS_INFO_STREAM("Front: " << temp_front_ << " **** Back: " << temp_back_);

  if (temp_front_ != "None")
  {
    front_laser_ = true;
  }
  if (temp_back_ != "None")
  {
    back_laser_ = true;
  }

  ROS_INFO("Front: %d **** Back: %d", front_laser_, back_laser_);
  GetParamUtil::GetParam("/sensor_safety_properties/lasers_enable", lasers_enable_, true);
  GetParamUtil::GetParam("/sensor_safety_properties/odometry_enable", odometry_enable_, true);
  GetParamUtil::GetParam("/sensor_safety_properties/encoder_enable", encoder_enable_, true);
  /* Jan 30th: Cameras disabled by default in sensor safety */
  GetParamUtil::GetParam("/sensor_safety_properties/camera_left_enable", camera_left_enable_, false);
  GetParamUtil::GetParam("/sensor_safety_properties/camera_right_enable", camera_right_enable_, false);

  if (front_laser_ == false || back_laser_ == false)
  {
    laser_number_ = 1;
  }
  else if (front_laser_ == true && back_laser_ == true)
  {
    laser_number_ = 2;
  }
  ROS_INFO("*** Sensor safety is configured with lasers of %d ***", laser_number_);
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void AvidbotsSensorSafety::ManagePubAndSub()
{
  // subscribe

  ROS_INFO_STREAM("*** Subscribing for sensor safety topic ***");

  if (laser_number_ == 0)
  {
    // do nothing
  }
  else if (laser_number_ == 1)
  {
    lasers_status_sub_    = node_handle_->
                        subscribe("/scan", 1,
                        &AvidbotsSensorSafety::LasersStatusCallback, this);
  }
  else if (laser_number_ == 2)
  {
    laser_front_sub_       = node_handle_->
                      subscribe("/scan_front", 1,
                      &AvidbotsSensorSafety::FrontLaserStatusCallback, this);
    laser_back_sub_       = node_handle_->
                      subscribe("/scan_back", 1,
                      &AvidbotsSensorSafety::BackLaserStatusCallback, this);
  }

  odometry_status_sub_    = node_handle_->
                      subscribe(avidbots_topics::front_wheel_odometry, 1,
                      &AvidbotsSensorSafety::OdometryStatusCallback, this);

  encoder_status_sub_    = node_handle_->
                      subscribe(avidbots_topics::mc_back_wheel_encoder_status, 1,
                      &AvidbotsSensorSafety::EncoderStatusCallback, this);

  camera_left_status_sub_    = node_handle_->
                      subscribe("/camera_left/depth/processed/points", 1,
                      &AvidbotsSensorSafety::CameraLeftStatusCallback, this);

  camera_right_status_sub_    = node_handle_->
                      subscribe("/camera_right/depth/processed/points", 1,
                      &AvidbotsSensorSafety::CameraRightStatusCallback, this);

  sensor_toggle_sub_       = node_handle_->
                      subscribe("/avidbots/ss/sensor_status", 1,
                      &AvidbotsSensorSafety::SensorToggleCallback, this);

  mcu_status_sub_     = node_handle_->subscribe(avidbots_topics::mcu_status_msg, 1,
                        &AvidbotsSensorSafety::MCUStatusCallBack, this);

  // publish
  ROS_INFO_STREAM("*** Publishing sensor safety update topic ***");
  sensor_safety_update_pub_        = node_handle_->advertise
                    <avidbots_msgs::diagnostics_states>
                    (avidbots_topics::diagnostics_states_topic, 1, true);

  ROS_INFO_STREAM("*** Publishing sensor safety status topic ***");
  sensor_safety_status_pub_        = node_handle_->advertise
                    <avidbots_msgs::sensor_safety_status>
                    (avidbots_topics::diagnostics_sensor_safety_topic, 1, true);
}

/**
 * @name    PublishSensorSafetyUpdate
 * @brief   Publish Sensor Safety Update
*/
void AvidbotsSensorSafety::PublishSensorSafetyUpdate()
{
  avidbots_msgs::diagnostics_states sensor_safety_state_msg;
  sensor_safety_state_msg.stamp = ros::Time::now();
  sensor_safety_state_msg.hardware_id = avidbots_msgs::diagnostics_states::kSensorSafety;
  sensor_safety_state_msg.description = "";
  if (lasers_status_ == SENSOR_ERROR || odometry_status_ == SENSOR_ERROR || encoder_status_ == SENSOR_ERROR
     || camera_left_status_ == SENSOR_ERROR || camera_right_status_ == SENSOR_ERROR)
  {
    sensor_safety_state_msg.states = avidbots_msgs::diagnostics_states::kStateError;
    if (lasers_status_ == SENSOR_ERROR)
    {
      sensor_safety_state_msg.description += "SENSORlaser.";
    }
    if (odometry_status_ == SENSOR_ERROR)
    {
      sensor_safety_state_msg.description += "SENSORodom.";
    }
    if (encoder_status_ == SENSOR_ERROR)
    {
      sensor_safety_state_msg.description += "SENSORenc.";
    }
    if (camera_left_status_ == SENSOR_ERROR)
    {
      sensor_safety_state_msg.description += "SENSORcl.";
    }
    if (camera_right_status_ == SENSOR_ERROR)
    {
      sensor_safety_state_msg.description += "SENSORcr.";
    }
  }
  else
  {
    sensor_safety_state_msg.states = avidbots_msgs::diagnostics_states::kStateOk;
  }
  sensor_safety_update_pub_.publish(sensor_safety_state_msg);
}

/**
 * @name    PublishSensorSafetyStatus
 * @brief   Publish Sensor Safety Status
*/
void AvidbotsSensorSafety::PublishSensorSafetyStatus()
{
  avidbots_msgs::sensor_safety_status sensor_safety_status_msg;
  sensor_safety_status_msg.stamp = ros::Time::now();
  if (lasers_status_ == SENSOR_ERROR || odometry_status_ == SENSOR_ERROR || encoder_status_ == SENSOR_ERROR
     || camera_left_status_ == SENSOR_ERROR || camera_right_status_ == SENSOR_ERROR)
  {
    sensor_safety_status_msg.status = avidbots_msgs::sensor_safety_status::kERROR;
    sensor_safety_status_msg.description += "ERROR:\n";
    if (lasers_status_ == SENSOR_ERROR)
    {
      sensor_safety_status_msg.description += "Laser.";
    }
    if (odometry_status_ == SENSOR_ERROR)
    {
      sensor_safety_status_msg.description += "Odometry.";
    }
    if (encoder_status_ == SENSOR_ERROR)
    {
      sensor_safety_status_msg.description += "Encoder.";
    }
    if (camera_left_status_ == SENSOR_ERROR || camera_right_status_ == SENSOR_ERROR)
    {
      sensor_safety_status_msg.description += "Camera.";
    }
  }
  else
  {
    sensor_safety_status_msg.status = avidbots_msgs::sensor_safety_status::kOK;
    sensor_safety_status_msg.description = "OK";
  }
  sensor_safety_status_pub_.publish(sensor_safety_status_msg);
}

/**
 * @name    ResetSensorSafetyStatus
 * @brief   Reset Sensor Safety Status
*/
void AvidbotsSensorSafety::ResetSensorSafetyStatus()
{
  lasers_status_ = SENSOR_OK;
  odometry_status_ = SENSOR_OK;
  encoder_status_ = SENSOR_OK;
  camera_left_status_ = SENSOR_OK;
  camera_right_status_ = SENSOR_OK;
}

/**
 * @name    MainTimerCallBack()
 * @brief   The main timer callback function that checks status
 */
void AvidbotsSensorSafety::MainTimerCallBack(const ros::TimerEvent &)
{
  if (!global_enable_)
  {
    ResetSensorSafetyStatus();
    PublishSensorSafetyUpdate();
    PublishSensorSafetyStatus();
    return;
  }

  // laser
  if (lasers_enable_)
  {
    if (laser_number_ == 1)
    {
      if (lasers_stopwatch_.elapsed() > ecl::TimeStamp(lasers_timeout_))
      {
        lasers_status_ = SENSOR_ERROR;
      }
      else
      {
        lasers_status_ = SENSOR_OK;
      }
    }
    else if (laser_number_ == 2)
    {
      if (laser_front_stopwatch_.elapsed() > ecl::TimeStamp(lasers_timeout_))
      {
        lasers_status_ = SENSOR_ERROR;
      }
      else if (laser_back_stopwatch_.elapsed() > ecl::TimeStamp(lasers_timeout_))
      {
        lasers_status_ = SENSOR_ERROR;
      }
      else
      {
        lasers_status_ = SENSOR_OK;
      }
    }
  }

  // odometry
  if (odometry_enable_)
  {
    if (mcu_state_ != MCU_STATE_VAR_NORMAL)
    {
      odometry_status_ = SENSOR_OK;
    }
    else if (odometry_stopwatch_.elapsed() > ecl::TimeStamp(odometry_timeout_))
    {
      odometry_status_ = SENSOR_ERROR;
    }
    else
    {
      odometry_status_ = SENSOR_OK;
    }
  }

  // encoder
  if (encoder_enable_)
  {
    if (encoder_stopwatch_.elapsed() > ecl::TimeStamp(encoder_timeout_))
    {
      encoder_status_ = SENSOR_ERROR;
    }
    else
    {
      encoder_status_ = SENSOR_OK;
    }
  }

  // camera

  // disable camera sensor safety check in release 1.3
  camera_left_enable_ = false;
  camera_right_enable_ = false;

  if (camera_left_enable_)
  {
    if (camera_left_stopwatch_.elapsed() > ecl::TimeStamp(camera_timeout_))
    {
      camera_left_status_ = SENSOR_ERROR;
    }
    else
    {
      camera_left_status_ = SENSOR_OK;
    }
  }
  if (camera_right_enable_)
  {
    if (camera_right_stopwatch_.elapsed() > ecl::TimeStamp(camera_timeout_))
    {
      camera_right_status_ = SENSOR_ERROR;
    }
    else
    {
      camera_right_status_ = SENSOR_OK;
    }
  }

  PublishSensorSafetyUpdate();
  PublishSensorSafetyStatus();
}

/**
 * @name    LasersStatusCallback
 * @brief   Lasers Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::LasersStatusCallback
                   (const sensor_msgs::LaserScanPtr &LaserScan)
{
  ecl::TimeStamp temp = laser_front_stopwatch_.elapsed();
  ROS_DEBUG_STREAM("*****Laser Stopwatch Elapsed: " << temp << std::endl);
  lasers_stopwatch_.restart();
}



/**
 * @name    FrontLaserStatusCallback
 * @brief   Front Laser Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::FrontLaserStatusCallback
                    (const sensor_msgs::LaserScanPtr &LaserScan)
{
  laser_front_stopwatch_.restart();
}

/**
 * @name    BackLaserStatusCallback
 * @brief   Back Laser Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::BackLaserStatusCallback
                    (const sensor_msgs::LaserScanPtr &LaserScan)
{
  laser_back_stopwatch_.restart();
}

/**
 * @name    OdometryStatusCallback
 * @brief   Odometry Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::OdometryStatusCallback
                   (const nav_msgs::OdometryPtr &Odometry)
{
  odometry_stopwatch_.restart();
}

/**
 * @name    EncoderStatusCallback
 * @brief   Encoder Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::EncoderStatusCallback
                   (const avidbots_msgs::back_wheel_encoder_statusPtr &back_wheel_encoder_status)
{
  encoder_stopwatch_.restart();
}

/**
 * @name    CameraStatusCallback
 * @brief   Camera Left Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::CameraLeftStatusCallback
                   (const avidbots_msgs::diagnostics_statesPtr &diagnostics_states)
{
  camera_left_stopwatch_.restart();
}

/**
 * @name    CameraRightStatusCallback
 * @brief   Camera Right Status Callback
 * @param[in]   diagnostics_states, const msg pointer for avidbots_msgs::diagnostics_states
 */
void AvidbotsSensorSafety::CameraRightStatusCallback
                   (const avidbots_msgs::diagnostics_statesPtr &diagnostics_states)
{
  camera_right_stopwatch_.restart();
}

/**
 * @name  SensorToggleCallback
 * @brief Call back for the sensor toggle msg.
 * @param[in] avidbots_msgs::sensor_toggle
 */
void AvidbotsSensorSafety::SensorToggleCallback
                   (const avidbots_msgs::sensor_toggle &sensor_toggle)
{
  switch (sensor_toggle.sensor_id)
  {
    case avidbots_msgs::sensor_toggle::kGlobal:
      global_enable_ = sensor_toggle.status;
      break;
    case avidbots_msgs::sensor_toggle::kLasers:
      lasers_enable_ = sensor_toggle.status;
      break;
    case avidbots_msgs::sensor_toggle::kOdometry:
      odometry_enable_ = sensor_toggle.status;
      break;
    case avidbots_msgs::sensor_toggle::kEncoder:
      encoder_enable_ = sensor_toggle.status;
      break;
    case avidbots_msgs::sensor_toggle::kCameraLeft:
      camera_left_enable_ = sensor_toggle.status;
      break;
    case avidbots_msgs::sensor_toggle::kCameraRight:
      camera_right_enable_ = sensor_toggle.status;
      break;
  }
}

/**
 * @name  MCUStatusCallBack
 * @brief Call back for the mcu status msg.
 * @param[in] mcu_status_msg: The MCU status message
 */
void AvidbotsSensorSafety::MCUStatusCallBack(const avidbots_msgs::mcu_status& mcu_status_msg)
{
  mcu_state_ = mcu_status_msg.state_variable;
}
