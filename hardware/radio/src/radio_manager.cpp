/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	radio_manager.cpp
 * @brief	Source File containing the RadioManager class
 * @author	Feng Cao
 */

#include <string>
#include "avidbots_radio/radio_manager.h"

/**
 * @name 	RadioManager
 * @brief	Default constructor
 */
RadioManager::RadioManager() :
snmp_mgr_(new SnmpManager(this)),
telnet_mgr_(new TelnetManager(this)),
util_mgr_(new UtilManager())
{
}

/**
 * @name  ~RadioManager
 * @brief Destructor
 */
RadioManager::~RadioManager() {}

/**
 * @name        Init()
 * @brief       Initialize ros interface
 * @param[in]   private_nh, node handler
 * @param[in]   node_handle, node handler
 */
int RadioManager::Init(const ros::NodeHandle& private_nh,
                                     ros::NodeHandle *node_handle)
{
  ROS_INFO_STREAM("RadioManager::Init: Initializing..");

  node_handle_  = node_handle;

  support_cnt_ = 0;

  if (snmp_mgr_->Init() == RADIO_SNMP_ERROR)
  {
    return -1;
  }

  ROS_INFO_STREAM("RadioManager::Init: Successfully initialized!");

  // Get parameters
  GetParamSettings(private_nh);

  // Initialize publishers and subscribers
  ManagePubAndSub();

  /* Create radio status timer */
  snmp_mgr_timer_ = node_handle_->createTimer
      (ros::Duration(15), &RadioManager::SnmpMgrTimerCallBack, this);
  telnet_mgr_timer_ = node_handle_->createTimer
      (ros::Duration(20), &RadioManager::TelnetMgrTimerCallBack, this);
  radio_status_timer_ = node_handle_->createTimer
      (ros::Duration(15), &RadioManager::RadioStatusTimerCallBack, this);

  return 0;
}

/**
 * @name  DeInit
 * @brief RadioManager DeInit
 */
int RadioManager::DeInit()
{
  snmp_mgr_->DeInit();
  telnet_mgr_->DeInit();

  delete snmp_mgr_;
  delete telnet_mgr_;
  delete util_mgr_;
}

/**
 * @name    GetParamSettings
 * @brief   Gets the parameters from param server
 * @param[in]   private_nh, node handler
 */
void RadioManager::GetParamSettings
            (const ros::NodeHandle& private_nh)
{
}

/**
 * @name  ManagePubAndSub
 * @brief Manages publishers and subscribers
 */
void RadioManager::ManagePubAndSub()
{
  // subscribe
  ROS_INFO_STREAM("*** Subscribing for radio settings topic ***");
  radio_settings_sub_    = node_handle_->
                      subscribe(avidbots_topics::radio_settings_topic, 1,
                      &RadioManager::RadioSettingsCallback, this);

  ROS_INFO_STREAM("*** Subscribing to Server Connection message ***");
  server_connection_subscriber_    = node_handle_->
                      subscribe(avidbots_topics::web_management_connected, 1,
                      &RadioManager::ServerConnectionStatusCallBack, this);

  // publish
  ROS_INFO_STREAM("*** Publishing radio response topic ***");
  radio_response_pub_        = node_handle_->advertise
                    <avidbots_msgs::radio_response>
                    (avidbots_topics::radio_response_topic, 1, true);

  ROS_INFO_STREAM("*** Publishing radio status topic ***");
  radio_status_pub_        = node_handle_->advertise
                    <avidbots_msgs::radio_settings>
                    (avidbots_topics::radio_status_topic, 1, true);
}

/**
 * @name    RadioSettingsCallback
 * @brief   Radio Settings Callback
 * @param[in]   settings, const msg pointer for avidbots_msgs::radio_settings
 */
void RadioManager::RadioSettingsCallback(const avidbots_msgs::radio_settingsPtr &settings)
{
  avidbots_msgs::radio_response response_msg;
  int ret = RADIO_SNMP_OK;
  ret = snmp_mgr_->ApplySettings(settings);
  if (ret != RADIO_SNMP_OK)
  {
    ROS_ERROR_STREAM("RadioManager::RadioSettingsCallback: Failed to apply the "
                     "requested settings: " << settings);
    response_msg.stamp = ros::Time::now();
    response_msg.result = ret;
    response_msg.description = "SNMP Set Error";
    PublishRadioResponse(response_msg);
    return;
  }
  ret = telnet_mgr_->TelnetProc(telnet_constants::telnet_proc_reset);
  if (ret == 0)
  {
    ROS_INFO_STREAM("RadioManager::RadioSettingsCallback: Successfully applied the "
                    "requested settings: " << settings);
    response_msg.description = "SNMP Set OK";
  }
  else
  {
    ROS_ERROR_STREAM("RadioManager::RadioSettingsCallback: Errorred while applying the "
                     "requested settings: " << settings);
    response_msg.description = "SNMP Set Commit Error";
  }
  response_msg.stamp = ros::Time::now();
  response_msg.result = ret;
  PublishRadioResponse(response_msg);
}

/**
 * @name        ServerConnectionStatusCallBack
 * @brief       Callback for server connection status
 * @param[in]   status_msg: true if connected
 */
void RadioManager::ServerConnectionStatusCallBack(const std_msgs::Bool& status_msg)
{
  ROS_DEBUG("ServerConnectionStatusCallBack: %d", status_msg.data);
}

/**
 * @name    SnmpMgrTimerCallBack()
 * @brief   The timer callback function for snmp manager
 */
void RadioManager::SnmpMgrTimerCallBack(const ros::TimerEvent & event)
{
  if (util_mgr_->Ping(RADIO_AGENT_IP))
  {
    if (snmp_mgr_->UpdateStatus(radio_status_msg_) == RADIO_SNMP_OK)
    {
      radio_status_msg_.support = true;
      if (support_cnt_ <= MAX_SUPPORT_CNT)
      {
        support_cnt_++;
      }
    }
    else
    {
      radio_status_msg_.support = false;
    }

    if (support_cnt_ > MAX_SUPPORT_CNT)
    {
      radio_status_msg_.support = true;
    }
  }
  if (util_mgr_->Ping("manage.avidbots.com"))
  {
    radio_status_msg_.wifi_status = "Connected";
  }
  else
  {
    radio_status_msg_.wifi_status = "Disconnected";
  }
}

/**
 * @name    TelnetMgrTimerCallBack()
 * @brief   The timer callback function for telnet manager
 */
void RadioManager::TelnetMgrTimerCallBack(const ros::TimerEvent & event)
{
  if (util_mgr_->Ping(RADIO_AGENT_IP))
  {
    telnet_mgr_->TelnetProc(telnet_constants::telnet_proc_status);
    telnet_mgr_->UpdateStatus(radio_status_msg_);
  }
}

/**
 * @name    RadioStatusTimerCallBack()
 * @brief   The timer callback function that updates the radio status
 */
void RadioManager::RadioStatusTimerCallBack(const ros::TimerEvent & event)
{
  PublishRadioStatus(radio_status_msg_);
}

/**
 * @name    PublishRadioResponse
 * @brief   Publish Radio Response
 * @param[in]   response_msg, const msg for avidbots_msgs::radio_response
*/
void RadioManager::PublishRadioResponse(const avidbots_msgs::radio_response &response_msg)
{
  radio_response_pub_.publish(response_msg);
}

/**
 * @name    PublishRadioStatus
 * @brief   Publish Radio Status
 * @param[in]   status_msg, const msg for avidbots_msgs::radio_settings
*/
void RadioManager::PublishRadioStatus(const avidbots_msgs::radio_settings &status_msg)
{
  radio_status_pub_.publish(status_msg);
}
