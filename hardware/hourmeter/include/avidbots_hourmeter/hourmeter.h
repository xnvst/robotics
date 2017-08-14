/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name	hourmeter.h
 * @brief	The main header for Hourmeter Class
 * @author	Pablo Molina
 */

#ifndef AVIDBOTS_HOURMETER_HOURMETER_H
#define AVIDBOTS_HOURMETER_HOURMETER_H

// ROS
#include <ros/ros.h>
#include "std_msgs/UInt64.h"

// C++
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

class Hourmeter
{
  public:
    /* Initializers */
    Hourmeter();
    void Init(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);

  private:
    /* ROS */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /* Timer */
    ros::Timer main_timer_;

    /* CallBacks */
    void MainTimerCallBack(const ros::TimerEvent &);

    /* Time variables */
    uint64_t current_time_;
    bool time_updated_externally_;

    /* Files */
    std::fstream time_file_;
    std::fstream new_time_file_;

    std::string time_file_name_;
    std::string new_time_file_name_;

    /* Helpers */
    void ReadTimeFile();
    void UpdateTimeFile();

    void ManagePubSub();
    void GetParam();

    void SetCurrentTimeCallBack(const std_msgs::UInt64& set_time);
    void SetCurrentTimeInSec(const uint64_t& new_time);

    /* Pub and Sub */
    ros::Publisher  time_pub_;
    ros::Subscriber set_time_sub_;
};

#endif  // AVIDBOTS_HOURMETER_HOURMETER_H
