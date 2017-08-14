/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	snmp_manager.h
 * @brief	Header file containing the SnmpManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_SNMP_MANAGER_H
#define AVIDBOTS_RADIO_SNMP_MANAGER_H

// CPP
#include <string>

// LCOAL
#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>
#include "avidbots_radio/snmp_constants.h"
#include "avidbots_radio/radio_manager.h"
#include <avidbots_msgs/radio_settings.h>
#include <avidbots_msgs/radio_response.h>

class RadioManager;

class SnmpManager
{
public:
  explicit SnmpManager(RadioManager* radio_mgr);
  ~SnmpManager();

  int Init();
  int DeInit();

  string ConvertIntToString(int64_t number);

  int ApplySettings(const avidbots_msgs::radio_settingsPtr &settings);
  int UpdateStatus(avidbots_msgs::radio_settings &status);

private:
  RadioManager* radio_mgr_;

  struct snmp_session read_session_, *read_ss_;
  struct snmp_session write_session_, *write_ss_;

  int SnmpGet(string param_oid, string &param_val);
  int SnmpSet(string param_oid, int param_type, int param_int_val, string param_str_val);
};

#endif  // AVIDBOTS_RADIO_SNMP_MANAGER_H

