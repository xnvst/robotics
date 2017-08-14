/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	radio_manager.h
 * @brief	Header file containing the RadioManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_RADIO_MANAGER_H
#define AVIDBOTS_RADIO_RADIO_MANAGER_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// CPP
#include <string>

// LCOAL
#include <avidbots_msgs/topics.h>
#include <avidbots_msgs/radio_settings.h>
#include <avidbots_msgs/radio_response.h>
#include "avidbots_radio/snmp_manager.h"
#include "avidbots_radio/telnet_manager.h"
#include "avidbots_radio/util_manager.h"

class SnmpManager;
class TelnetManager;
class UtilManager;

class RadioManager
{
public:
  static RadioManager& GetInstance()
  {
    static RadioManager instance;
    return instance;
  }
  ~RadioManager();

  int Init(const ros::NodeHandle& private_nh,
                  ros::NodeHandle* node_handle);
  int DeInit();

  void GetParamSettings(const ros::NodeHandle& private_nh);
  void ManagePubAndSub();

  // CallBacks
  void RadioSettingsCallback(const avidbots_msgs::radio_settingsPtr &settings);
  void ServerConnectionStatusCallBack(const std_msgs::Bool& status_msg);

  /* Timer callbacks */
  void SnmpMgrTimerCallBack(const ros::TimerEvent&);
  void TelnetMgrTimerCallBack(const ros::TimerEvent&);
  void RadioStatusTimerCallBack(const ros::TimerEvent&);

  // Publishers
  void PublishRadioResponse(const avidbots_msgs::radio_response &response_msg);
  void PublishRadioStatus(const avidbots_msgs::radio_settings &status_msg);

private:
  RadioManager();

  SnmpManager *snmp_mgr_;
  TelnetManager *telnet_mgr_;
  UtilManager *util_mgr_;

  ros::NodeHandle*  node_handle_;

  ros::Timer snmp_mgr_timer_,
             telnet_mgr_timer_,
             radio_status_timer_;

  // ROS publisher/subscriber
  ros::Subscriber   radio_settings_sub_,
                    server_connection_subscriber_;
  ros::Publisher    radio_response_pub_,
                    radio_status_pub_;

  avidbots_msgs::radio_settings radio_status_msg_;
  int support_cnt_;
};

#define RadioManagerInstance \
  RadioManager::GetInstance()

#define MAX_SUPPORT_CNT   2

#endif  // AVIDBOTS_RADIO_RADIO_MANAGER_H
