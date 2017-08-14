/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	hourmeter.cpp
 * @brief	The main implementation file for the hourmeter
 * @author	Pablo Molina
 */


// Local
#include "avidbots_hourmeter/hourmeter.h"

// C++
#include <string>
#include <iostream>

// External
#include "avidbots_library/get_param/get_param_util.h"
#include "avidbots_msgs/topics.h"



/**
 * @name    Hourmeter()
 * @brief   Default constructor
 */
Hourmeter::Hourmeter()
{
  // Setting up the defaults
  time_file_name_         = "/data/hourmeter/time.txt";
  new_time_file_name_     = time_file_name_ + ".new";
  time_updated_externally_           = false;
}

/**
 * @name    Init()
 * @brief   Initializes class
 * param[in] private_nh: Private node handle
 */
void Hourmeter::Init(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
{
  private_nh_ = private_nh;
  nh_         = nh;

  GetParam();
  // manage pub_sub
  ManagePubSub();

  /* Create main timer */
  main_timer_ = private_nh_.createTimer
      (ros::Duration(1.0), &Hourmeter::MainTimerCallBack, this);
}

/**
 * @name    SetCurrentTimeCallBack()
 * @brief   The callback to set the time of the hourmeter externally
 * param[in] set_time, the actual time to set.
 */
void Hourmeter::SetCurrentTimeCallBack(const std_msgs::UInt64& set_time)
{
  SetCurrentTimeInSec(set_time.data);
  time_updated_externally_ = true;
}

/**
 * @name    ManagePubSub()
 * @brief   Manages the Publishers and Subscribers to this node
 */
void Hourmeter::ManagePubSub()
{
  // Publish
  ROS_DEBUG_STREAM("*** Publishing hourmeter time in seconds ***");
  time_pub_     = nh_.advertise
                       <std_msgs::UInt64>
                       (avidbots_topics::hourmeter_time, 1, true);
  // Subscribe
  ROS_DEBUG_STREAM("*** Subscribing to setting hourmeter time ***");
  set_time_sub_ = nh_.subscribe(avidbots_topics::hourmeter_set_time, 1,
                         &Hourmeter::SetCurrentTimeCallBack, this);
}

/**
 * @name    GetParam()
 * @brief   Gets the parameters related to this node
 */
void Hourmeter::GetParam()
{
  GetParamUtil::GetParam("/hourmeter_properties/file_name", time_file_name_, "/data/hourmeter/time.txt");
  new_time_file_name_ = time_file_name_ + ".new";
}

/**
 * @name    MainTimerCallBack()
 * @brief   Main timer callback to update internal time
 */
void Hourmeter::MainTimerCallBack(const ros::TimerEvent &)
{
  if (!time_updated_externally_)
  {
    // read file, close file
    ReadTimeFile();
    ROS_DEBUG_STREAM("The time in the file is: "<< current_time_);
  }
  // increment +1 sec
  current_time_ += 1;
  // Update file
  UpdateTimeFile();
  // Publish current time
  std_msgs::UInt64 current_time_msg;
  current_time_msg.data = current_time_;
  time_pub_.publish(current_time_msg);
  // resetting external update flag
  time_updated_externally_ = false;
}

/**
 * @name    ReadTimeFile()
 * @brief   This functions reads from the time file
 */
void Hourmeter::ReadTimeFile()
{
  // pre-allocating some memory
  std::string data;
  bool create_new_file = false;

  try
  {
    // Opening file
    time_file_.open(time_file_name_);
  }
  catch(std::exception const & e)
  {
    ROS_ERROR("There was a problem opening the time.txt file");
    create_new_file = true;
  }

  try
  {
    // Read data
    time_file_ >> data;
    // Close the file
    time_file_.close();
    // Attempt to convert data to int
    current_time_ = (uint64_t) std::stol(data);
  }
  catch(std::exception const & e)
  {
    ROS_ERROR("There was an error reading the time file.");
    create_new_file = true;
  }

  if (create_new_file)
  {
    // Reset timer, and create new file
    current_time_ = 0;
    time_file_.close();
    time_file_.open(time_file_name_, std::ios_base::out | std::ios::trunc);
    time_file_ << current_time_;
    time_file_.close();
  }
}

/**
 * @name    UpdateFile()
 * @brief   This function updates the time file
 */
void Hourmeter::UpdateTimeFile()
{
  /* The goal of this function is:
   * - Create new file new_time
   * - Move new_file as time.txt
   * This function assumes that "rename" will destroy the old file values.
   * This is true in C++11 and GCC.
   * If this code is to be used under other toolchain, the rename function must be understood
   */

  try
  {
    // Create new file
    // Ensure to truncate the file
    new_time_file_.open(new_time_file_name_, std::ios_base::out | std::ios::trunc);
    new_time_file_ << current_time_;
    new_time_file_.close();
    // Move time.txt.new to time.txt
    if (rename(new_time_file_name_.c_str(), time_file_name_.c_str()) != 0)
    {
      ROS_ERROR("Unable to rename time.txt.new to time.txt");
    }
  }
  catch(std::exception const & e)
  {
    ROS_ERROR("There was an error updating the file. Skipping this writting cycle...");
  }
}

/**
 * @name    SetCurrentTimeInSec()
 * @brief   Set the main time variable
 * param[in]  new_time: the new time
 */
void Hourmeter::SetCurrentTimeInSec(const uint64_t &new_time)
{
  current_time_ = new_time;
}

