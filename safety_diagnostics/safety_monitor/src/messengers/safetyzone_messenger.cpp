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
 * @brief safetyzone_messenger cpp file contains the safetyzone_messenger
 * @author Keshav Iyengar
 */


// LOCAL
#include <avidbots_msgs/topics.h>
#include "avidbots_safety_monitor/sm_ros_interface.h"
#include "avidbots_safety_monitor/messengers/safetyzone_messenger.h"
#include "avidbots_safety_monitor/safety_zone/sm_expsafetyzone.h"
#include "avidbots_safety_monitor/safety_zone/sm_dangerzone.h"

#include <avidbots_library/geometry/geometry_msgs/geometry_util.h>
#include <avidbots_library/get_param/get_param_util.h>
#include <avidbots_library/socket_can/avidbots_can_open_constants.h>

/**
 * @name        SafetyZoneMessenger
 * @brief       Constructor
 * @param[in]   ros: a pointer to the ROSInterface it communicates with
 */
SafetyZoneMessenger::SafetyZoneMessenger(ROSInterface* ros)
{
  ros_ = ros;
}

/**
 * @name        SafetyZoneMessenger
 * @brief       Destructor
 */
SafetyZoneMessenger::~SafetyZoneMessenger()
{
  delete danger_zone_;
  delete exp_safetyzone_;
}

/**
 * @name        initialize
 * @brief       initialize publishers and subscribers used
 *                commonly by Wheely and Shiny to communicate with ROS
 */
void SafetyZoneMessenger::Initialize()
{
  ROS_INFO("*** Initializing Safety Monitor Safety Zone Messenger ***");
  has_laser_tf_ = false;

  danger_zone_ = new SMDangerZone();
  exp_safetyzone_ = new SMExpSafetyZone();

  danger_zone_status_ = NO_ERROR;
  exp_safetyzone_status_ = NO_ERROR;
  local_controller_safety_status_ = NO_ERROR;

  danger_zone_->Initialize();
  exp_safetyzone_->Initialize();

  GetParamSettings();
  InitializePublishersAndSubscribers();
}

/**
 * @name        GetParamSettings
 * @brief       Sets parameter data members according to values from ROS param server
 */
void SafetyZoneMessenger::GetParamSettings()
{
  ROS_INFO_STREAM("*** Safety Zone Messenger getting Parameters ***");
  GetParamUtil::GetParam("/sm_properties/enable_danger_safetyzone", enable_danger_zone_, false);
  ros_->manager_->SetEnableDangerZone(enable_danger_zone_);
  GetParamUtil::GetParam("/sm_properties/enable_danger_zone_zero_vel", enable_dangerzone_zero_vel_, false);
  ros_->manager_->SetDangerZoneZeroVelEnabled(enable_dangerzone_zero_vel_);
  GetParamUtil::GetParam("/sm_properties/danger_obstacle_timeout", obstacle_timeout_, 1.5);
  ros_->manager_->SetObstacleTimeout(obstacle_timeout_);
  GetParamUtil::GetParam("/sm_properties/disable_danger_obstacle_reset", skip_danger_obstacle_reset_, false);
  ros_->manager_->SetSkipDangerObstacleReset(skip_danger_obstacle_reset_);
  GetParamUtil::GetParam("/sm_properties/enable_expanding_safetyzone", enable_exp_safety_zone_, false);
  ros_->manager_->SetEnableExpSafetyZone(enable_exp_safety_zone_);
}

/**
 * @name  InitializePublishersAndSubscribers
 * @brief Manages publishers and subscribers
 */
void SafetyZoneMessenger::InitializePublishersAndSubscribers()
{
  ROS_INFO_STREAM("*** Safety Zone Messenger Initializing subs and pubs ***");

  ROS_DEBUG_STREAM("*** Subscribing to the enable safety zone message ***");
  enable_zones_sub_   = node_.subscribe(avidbots_topics::sm_enable_safety_zones, 1,
                        &SafetyZoneMessenger::EnableZonesCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the laser scan message ***");
  laser_sub_          = node_.subscribe(avidbots_topics::raw_laser_topic, 1,
                        &SafetyZoneMessenger::LaserScanCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the twist command message ***");
  twist_cmd_sub_      = node_.
                        subscribe(avidbots_topics::cmd_vel_mux_output, 1,
                        &SafetyZoneMessenger::TwistCmdCallBack, this);

  ROS_DEBUG_STREAM("*** Subscribing to the local controller safety_check message ***");
  local_controller_safety_check_sub_      = node_.
                        subscribe(avidbots_topics::local_controller_safety_check, 1,
                        &SafetyZoneMessenger::LocalControllerSafetyCheckCallBack, this);

  ROS_DEBUG_STREAM("*** Publishing Danger Obstacle status command message ***");
  danger_obstacle_pub_ = node_.advertise<std_msgs::Bool>
                         (avidbots_topics::danger_obstacle_status_topic, 10, true);

  ROS_DEBUG_STREAM("*** Publishing  Laser Point Cloud ***");
  laser_pub_          = node_.advertise<PointCloud>("/safetymonitor/laserPointCloud", 10, true);

  ROS_DEBUG_STREAM("*** Publishing the ROS visualization message ***");
  danger_vis_pub_     = node_.advertise<visualization_msgs::Marker>("danger_safetyzone_vis", 10);

  ROS_DEBUG_STREAM("*** Publishing the ROS visualization message ***");
  vel_vis_pub_        = node_.advertise<visualization_msgs::Marker>("vel_safetyzone_vis", 10);

  ROS_DEBUG("*** Publishing Manual Override Mode Cmd message ***");
  manual_override_cmd_publisher_ = node_.advertise<std_msgs::Bool>
      (avidbots_topics::mc_manual_override_msg, 10, true);
}

/**
 * @name  InitializeTimers
 * @brief Manages Timers
 */
void SafetyZoneMessenger::InitializeTimers()
{
  ROS_INFO_STREAM("*** Safety Zone Messenger Initializing timers ***");

  /* Create safetyzone timer */
  safetyzone_timer_ = node_.createTimer
      (ros::Duration(0.05), &SafetyZoneMessenger::SafetyZoneTimerCallBack, this);
}

////////////////
/* CallBacks */
//////////////

/**
 * @name  SafetyZoneTimerCallBack
 * @brief Call back for updating the manager on status
 */
void SafetyZoneMessenger::SafetyZoneTimerCallBack(const ros::TimerEvent&)
{
  if (ros_->manager_->GetEnableSafetyMonitor())
  {
    ros_->manager_->SetExpSafetyZoneStatus(exp_safetyzone_status_);
    ros_->manager_->SetDangerZoneStatus(danger_zone_status_);

    if (!ros_->manager_->GetExpSafetyZoneStatus() && ros_->manager_->GetDangerZoneStatus())
    {
      ros_->manager_->SMStateEvent(EXP_SAFETYZONE_EVENT);
    }
    if (!ros_->manager_->GetDangerZoneStatus())
    {
      ros_->manager_->SMStateEvent(DANGERZONE_EVENT);
    }
  }
  else
  {
    ros_->manager_->SetExpSafetyZoneStatus(NO_ERROR);
    ros_->manager_->SetDangerZoneStatus(NO_ERROR);
  }
}

/**
 * @name  TwistCmdCallBack
 * @brief Call back for the twist cmd msg.
 * @param[in] vel_msg: The values of the commanded velocity
 */
void SafetyZoneMessenger::TwistCmdCallBack(const geometry_msgs::TwistPtr& vel_msg)
{
  /* For any velocity to be sent to safety_zone_, curr_state must be idle */
  if (ros_->manager_->GetState() == IDLE_STATE)
  {
    ang_vel_ = vel_msg->angular.z;
     /* Decrease safety zone when making slow turns */
    if (fabs(ang_vel_) > 0.6 &&
        fabs(vel_msg->linear.x) < 1.0 &&
        fabs(vel_msg->linear.y) < 1.0)
    {
      RetractSafetyZone(velocity_vector_, 0.025);
    }
    else
    {
      velocity_vector_.x = vel_msg->linear.x;
      velocity_vector_.y = vel_msg->linear.y;
    }
  }
  else if (ros_->manager_->GetState() == EXP_SAFETYZONE_STATE)
  {
    /* Gradually retract safety zone */
    RetractSafetyZone(velocity_vector_, 0.025);
    ang_vel_ = 0.0;
  }
  else
  {
    velocity_vector_.x = 0.0;
    velocity_vector_.y = 0.0;
    ang_vel_ = 0.0;
  }
}

/*
 * @name        EnableZonesCallBack
 * @brief       Callback for enabling safety zone
 * @param[in]   enable_zones_msg: The disabling safety zone msg
 *              true for safety zone enabled, false for safety zone disabled
 */
void SafetyZoneMessenger::EnableZonesCallBack(const std_msgs::Bool& enable_zones_msg)
{
  enable_safety_zone_monitor_ = enable_zones_msg.data;
  ros_->manager_->SetEnableExpSafetyZone(enable_safety_zone_monitor_);
  ros_->manager_->SetEnableDangerZone(enable_safety_zone_monitor_);
}

/**
 * @name        LaserScanCallBack
 * @brief       Callback function for laser scan
 * @param[in]   scan: The values of the scan from laser
 */

void SafetyZoneMessenger::LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (ros_->manager_->GetEnableSafetyMonitor())
  {
    if (!has_laser_tf_)
    {
      if (WaitForLaserTransform(scan))
      {
        has_laser_tf_ = true;
        danger_zone_->InitLaserInfo();
      }
      else
      {
        ROS_ERROR("Transformed failed, retrying during next callback.");
        return;
      }
    }
    else
    {
      PointCloud::Ptr laser_cloud;
      laser_cloud = GetLaserPointCloud(scan);
      if (laser_cloud != NULL)
      {
        exp_safetyzone_->CreateExpSafetyZone(velocity_vector_, ang_vel_);
        exp_safetyzone_status_ = exp_safetyzone_->ZoneClear(laser_cloud,
                                               ros_->manager_->GetExpSafetyZoneStatus()) ? NO_ERROR : ERROR;
        vel_vis_pub_.publish(exp_safetyzone_->GetExpZoneMarker());
        danger_zone_->CreateDangerZone();
        danger_zone_status_ = danger_zone_->ZoneClear(laser_cloud,
                                            ros_->manager_->GetDangerZoneStatus()) ? NO_ERROR : ERROR;
        danger_vis_pub_.publish(danger_zone_->GetDangerZoneMarker());
      }
    }
  }
}

/*
 * @name        LocalControllerSafetyCheckCallBack
 * @brief       Callback for local controller safety check
 * @param[in]   safety_check_msg: The disabling safety zone msg
 */
void SafetyZoneMessenger::LocalControllerSafetyCheckCallBack(const avidbots_msgs::safety_check& safety_check_msg)
{
  if (safety_check_msg.stuck)
  {
    ros_->manager_->SetLocalControllerSafetyStatus(ERROR);
  }
  else
  {
    ros_->manager_->SetLocalControllerSafetyStatus(NO_ERROR);
  }
}

///////////////////////
/* Helper Functions */
//////////////////////

/**
 * @name      WaitForLaserTransform()
 * @brief     Waits for the transform from laser frame to base link frame
 * @param[in] scan: scan message from the urg node
 */
bool SafetyZoneMessenger::WaitForLaserTransform(
  const sensor_msgs::LaserScan::ConstPtr& scan)
{
  return  listener_.waitForTransform(
          scan->header.frame_id, "/base_link",
          scan->header.stamp +
          ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
          ros::Duration(1.0));
}

/**
 * @name    GetLaserPointCloud
 * @brief   Returns cloud points (cartesian coordinates)
            from laser points (polar coordinates)
 * @param[in] scan: The values of the scan from laser
 */
PointCloud::Ptr SafetyZoneMessenger::GetLaserPointCloud(
  const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::PointCloud2 cloud;
  PointCloud::Ptr laser_cloud(new PointCloud);
  listener_.waitForTransform(
        scan->header.frame_id, "/base_link",
        scan->header.stamp +
        ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        ros::Duration(1.0));
  projector_.transformLaserScanToPointCloud("/base_link", *scan, cloud, listener_);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);
  return laser_cloud;
}

/**
 * @name    RetractSafetyZone()
 * @brief   Decreases the x and y velocity of a given vector.
 * param[in] velocity_vector: A velocity vector
 * param[in] val: The value to decrease the x and y components of the vector
 */
void SafetyZoneMessenger::RetractSafetyZone(Point& velocity_vector, double val)
{
  /* Retract x velocity */
  velocity_vector.x -= val;
  if (velocity_vector.x < 0.0)
  {
    velocity_vector.x = 0.0;
  }

  /* Retract y velocity */
  if (velocity_vector.y > 0.0)
  {
    velocity_vector.y -= val;
    if (velocity_vector.y < 0.0)
    {
      velocity_vector.y = 0.0;
    }
  }
  else if (velocity_vector.y < 0.0)
  {
    velocity_vector.y += val;
    if (velocity_vector.y > 0.0)
    {
      velocity_vector.y = 0.0;
    }
  }
}

/**
 * @name    PublishDangerZoneObstacle()
 * @brief   Publishes true/false if an obstacle is the danger zone
 * param[in] status: boolean of whether an obstacle is the danger zone 
 */
void SafetyZoneMessenger::PublishDangerZoneObstacle(bool status)
{
  std_msgs::Bool danger_obstacle_status_msg;
  danger_obstacle_status_msg.data = status;
  danger_obstacle_pub_.publish(danger_obstacle_status_msg);
}

/**
 * @name    PublishMCUOutputCommand()
 * @brief   Sends an estate command to MCU driver via CANnetwork with
 *          value to indicate safety zone cause error.
 */
void SafetyZoneMessenger::PublishMCUOutputCommand()
{
  /* Publish estop state in CAN */
  ROS_ERROR("MCU Driver has been set to Emergency State by SafetyZone");

  ros_->PublishMCUOutputCommand(SAFETY_MONITOR_SAFETY_ZONE_FAIL);
}
