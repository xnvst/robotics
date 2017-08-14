/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 * @name	simulated_free_wheel.cpp
 * @brief	Source file for simulating a free spinning wheel for odometry
 * @author	Pablo Molina
 */
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <avidbots_msgs/free_wheel_status.h>
#include <avidbots_msgs/topics.h>
#include <avidbots_library/get_param/get_param_util.h>
#include <std_msgs/Int64.h>

#include "boost/thread/mutex.hpp"

/**  Constants **/
double k_wheel_radius_meters  = 0.0762;
int k_ticks_per_rev           = 2048;
double k_pos_y_free_wheel     = 0.26;
bool k_use_free_wheel         = true;
const double delta_time       = 0.01;
bool k_invert_free_wheel      = false;

/**  Global variables **/
geometry_msgs::Pose g_prev_gazebo_pose;
geometry_msgs::Pose g_gazebo_pose;
geometry_msgs::Twist g_gazebo_twist;
int g_gazebo_id_state = 0;
int g_tick_count = -1;
double g_invert_free_wheel = 1.0;
ros::Publisher tick_stamped_publisher;
ros::Publisher tick_publisher;
ros::Time g_old_time;
ros::Time g_new_time;

bool g_new_gazebo_msg_rcvd_ = false;

boost::mutex g_gazebo_states_mutex;


/**
 * @name  ConvertDeltaPosToTicks
 * @brief Convert the delta position to ticks
 * @param[in] delta_x: The delta position
 */
int ConvertDeltaPosToTicks(const double& delta_x)
{
  /*
   * 1. Take delta position and convert it to radians
   * 2. Convert radians to ticks
   */
  double rad = delta_x / k_wheel_radius_meters;
  return (rad * k_ticks_per_rev)/(2*M_PI);
}

/**
 * @name  ConvertDeltaPosToTicks
 * @brief Convert the delta position to ticks
 * @param[in] delta_x: The delta position
 */
int ConvertWToDeltaTicks()
{
  /*
   * 1. Take w and convert it to Vwx
   * 2. Convert Vwx to delta ticks
   */
  double Vwx = g_gazebo_twist.angular.z * k_pos_y_free_wheel;
  double delta_travel_due_w = Vwx * delta_time;
  return ConvertDeltaPosToTicks(delta_travel_due_w);
}

/**
 * @name  PublishTicksCallBack
 * @brief This function publishes the ticks
 * @param[in] timer_event: The timer event
 */
void PublishTicksCallBack(const ros::TimerEvent& timer_event)
{
  boost::mutex::scoped_lock(g_gazebo_states_mutex);

  /*
   * 1. Computes delta position
   * 2. Converts delta position to ticks
   * 3. Computes ticks due to w
   * 4. Publishes ticks
   */
  if (g_tick_count == -1)
  {
    g_prev_gazebo_pose = g_gazebo_pose;
    g_tick_count = 0;
  }

  double dt = g_new_time.toSec() - g_old_time.toSec();

  if (!g_new_gazebo_msg_rcvd_ || dt == 0.0)
    return;

  double current_heading = tf::getYaw(g_prev_gazebo_pose.orientation);

  double delta_x_world_frame =  g_gazebo_pose.position.x -
                                g_prev_gazebo_pose.position.x;
  double delta_y_world_frame =  g_gazebo_pose.position.y -
                                g_prev_gazebo_pose.position.y;

  double delta_x_robot_frame = (delta_x_world_frame * cos(current_heading))
      + (delta_y_world_frame * sin(current_heading));

//  ROS_INFO("t=%.3f,  dt=%.3f,  dx_L=%.4f,  dx_G=%.4f,  dy_G=%.4f,  v=%.4f",
//           ros::Time::now().toSec(), dt, delta_x_robot_frame,
//           delta_x_world_frame, delta_y_world_frame,
//           delta_x_robot_frame / dt);

  avidbots_msgs::free_wheel_status msg_out_stamped;
  g_tick_count += ConvertDeltaPosToTicks(delta_x_robot_frame);
  g_tick_count += ConvertWToDeltaTicks();
  msg_out_stamped.stamp = g_new_time;
  msg_out_stamped.ticks = g_tick_count * g_invert_free_wheel;
  tick_stamped_publisher.publish(msg_out_stamped);

  std_msgs::Int64 msg_out;
  msg_out.data = g_tick_count * g_invert_free_wheel;
  tick_publisher.publish(msg_out);

  g_prev_gazebo_pose = g_gazebo_pose;
  g_old_time = g_new_time;

  g_new_gazebo_msg_rcvd_ = false;
}

/**
 * @name  gazeboCallBack
 * @brief The gazebo callback
 * @param[in] msg_in: The gazebo status message
 */
void gazeboCallBack
        (const gazebo_msgs::ModelStates::Ptr& msg_in)
{
  boost::mutex::scoped_lock(g_gazebo_states_mutex);

  // Check what id in the vector belongs to "mobile_base"
  // Do this only once and skip this afterwards
  if (g_gazebo_id_state == 0)
  {
    for (int i = 0; i < msg_in->name.size(); i++)
    {
      if (msg_in->name.at(i) == "mobile_base")
      {
        g_gazebo_id_state = i;
        break;
      }
    }
  }

  // ROS_INFO("Got a Gazebo status message");
  g_gazebo_pose.position = msg_in->pose[g_gazebo_id_state].position;
  g_gazebo_pose.orientation = msg_in->pose[g_gazebo_id_state].orientation;
  g_gazebo_twist = msg_in->twist[g_gazebo_id_state];
  g_new_time = ros::Time::now();
  g_new_gazebo_msg_rcvd_ = true;
//  ROS_INFO_THROTTLE(1.0, "GazeboCallBack: vx=%.3f, vy=%.3f, vth=%.3f",
//                    g_gazebo_twist.linear.x, g_gazebo_twist.linear.y,
//                    g_gazebo_twist.angular.z);
//  ROS_WARN("t=%.3f,  dt=%.3f,  x=%.4f,  y=%.4f", g_new_time.toSec(),
//           g_new_time.toSec() - g_old_time.toSec(),
//           g_gazebo_pose.position.x, g_gazebo_pose.position.y);
}


// ------
//  MAIN
// ------
int main(int argc, char** argv)
{
  // Creating ROS variables
  ros::init(argc, argv, "simulated_free_wheel",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle node;

  GetParamUtil::GetParam("/free_wheel_properties/use_free_wheel",
                         k_use_free_wheel, false);
  GetParamUtil::GetParam("/free_wheel_properties/radius_free_wheel",
                         k_wheel_radius_meters, 0.0762);
  GetParamUtil::GetParam("/free_wheel_properties/ticks_per_rev_free_wheel",
                         k_ticks_per_rev, 2048);
  GetParamUtil::GetParam("/free_wheel_properties/pos_y_free_wheel",
                         k_pos_y_free_wheel, 0.26);
  GetParamUtil::GetParam("/free_wheel_properties/invert_free_wheel",
                         k_invert_free_wheel, false);
  g_invert_free_wheel = k_invert_free_wheel ? -1.0 : 1.0;

  // exit node if not needed
  if (!k_use_free_wheel)
  {
    ROS_INFO_STREAM("Exiting free wheel simulator");
    return 0;
  }

  // Wait until Gazebo is up and running
  sleep(10);

  // Publishing and subscribing to messages
  ROS_DEBUG_STREAM("*** Publishing free wheel ticks (stamped) ***");
  tick_stamped_publisher = node.advertise
                    <avidbots_msgs::free_wheel_status>
                    ("/arduino/encoder_stamped", 1, false);

  ROS_DEBUG_STREAM("*** Publishing free wheel ticks ***");
  tick_publisher = node.advertise
                    <std_msgs::Int64>
                    (avidbots_topics::free_wheel_topic, 1, false);

  ROS_DEBUG_STREAM("*** Subscribing to the gazebo model states ***");
  ros::Subscriber gazebo_sub = node.subscribe("/gazebo/model_states", 1,
                           &gazeboCallBack);

  sleep(1);

  ros::Timer publish_data_timer_ = node.createTimer
      (ros::Duration(delta_time), &PublishTicksCallBack);

  ros::spin();
}
