/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	SNMP Constants header file
 * @brief	SNMP Constants
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_SNMP_CONSTANTS_H
#define AVIDBOTS_RADIO_SNMP_CONSTANTS_H

#include <string>
#include <net-snmp/library/asn1.h>

#define USE_SNMP_VERSION_3

enum SNMP_ERROR_CODE
{
  RADIO_SNMP_OK = 0,
  RADIO_SNMP_ERROR = 1,
};

const std::string RADIO_AGENT_IP = "192.168.3.1";

const unsigned char COMMUNITY_READ_STR[16] = "public";
const unsigned char COMMUNITY_WRITE_STR[16] = "private";

const std::string SNMP_OID_PREFIX = ".1.3.6.1.4.1.21703.7000";
const std::string SNMP_COMMAND_OID_PREFIX = "iso.3.6.1.4.1.21703.7000";

// WIFI
const std::string WIFI_VIRTUAL_INTERFACEI_1_SSID_OID = ".3.2.1.0";
const int WIFI_VIRTUAL_INTERFACEI_1_SSID_TYPE = ASN_OCTET_STR;

const std::string WIFI_VIRTUAL_INTERFACEI_1_ENCRYPTION_OID = ".3.2.11.0";
const int WIFI_VIRTUAL_INTERFACEI_1_ENCRYPTION_TYPE = ASN_INTEGER;

const std::string WIFI_VIRTUAL_INTERFACEI_1_KEY_OID = ".3.2.12.0";
const int WIFI_VIRTUAL_INTERFACEI_1_KEY_TYPE = ASN_OCTET_STR;

const std::string WIFI_SUBMIT_OID = ".3.10.200.0";
const int WIFI_SUBMIT_TYPE = ASN_INTEGER;

// CELL
const std::string CARRIER_CONNECT1_APN_OID = ".5.2.2.0";
const int CARRIER_CONNECT1_APN_TYPE = ASN_OCTET_STR;

const std::string CARRIER_CONNECT1_USERNAME_OID = ".5.2.4.0";
const int CARRIER_CONNECT1_USERNAME_TYPE = ASN_OCTET_STR;

const std::string CARRIER_CONNECT1_PASSWD_OID = ".5.2.5.0";
const int CARRIER_CONNECT1_PASSWD_TYPE = ASN_OCTET_STR;

const std::string CARRIER_TMPSTATUS_CONNECT_STATUS_OID = ".5.4.9.0";
const int CARRIER_TMPSTATUS_CONNECT_STATUS_TYPE = ASN_OCTET_STR;

const std::string CARRIER_TMPSTATUS_RSSI_OID = ".5.4.5.0";
const int CARRIER_TMPSTATUS_RSSI_TYPE = ASN_OCTET_STR;

#ifdef USE_SNMP_VERSION_3
const std::string snmp_v3_passphrase = "xn2RGYMlFLyCdTtbzic1";
#endif

#endif  // AVIDBOTS_RADIO_SNMP_CONSTANTS_H
