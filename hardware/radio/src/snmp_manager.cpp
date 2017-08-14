/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	snmp_manager.cpp
 * @brief	Source File containing the SnmpManager class
 * @author	Feng Cao
 */

// ROS
#include <ros/ros.h>

// CPP
#include <string>
#include <stdexcept>

// LOCAL
#include "avidbots_radio/snmp_manager.h"
#include "avidbots_radio/util_manager.h"

/**
 * @name 	SnmpManager
 * @brief	Default constructor
 */
SnmpManager::SnmpManager(RadioManager* radio_mgr) : radio_mgr_(radio_mgr)
{
}

/**
 * @name  ~SnmpManager
 * @brief Destructor
 */
SnmpManager::~SnmpManager() {}

/**
 * @name  Init
 * @brief SnmpManager Init
 * @param[in] agentIP: agent IP address
 */
int SnmpManager::Init()
{
  init_snmp("snmpapp");

  snmp_sess_init(&read_session_);
  read_session_.peername = const_cast<char *>(RADIO_AGENT_IP.c_str());

  snmp_sess_init(&write_session_);
  write_session_.peername = const_cast<char *>(RADIO_AGENT_IP.c_str());

#ifdef USE_SNMP_VERSION_3

  /* set the SNMP version number */
  read_session_.version = SNMP_VERSION_3;

  /* set the SNMPv3 user name */
  read_session_.securityName = strdup("userV3");
  read_session_.securityNameLen = strlen(read_session_.securityName);

  /* set the security level to authenticated, but not encrypted */
  read_session_.securityLevel = SNMP_SEC_LEVEL_AUTHNOPRIV;

  /* set the authentication method to MD5 */
  read_session_.securityAuthProto = usmHMACMD5AuthProtocol;
  read_session_.securityAuthProtoLen = sizeof(usmHMACMD5AuthProtocol)/sizeof(oid);
  read_session_.securityAuthKeyLen = USM_AUTH_KU_LEN;

  /* set the authentication key to a MD5 hashed version of our
     passphrase "The Net-SNMP Demo Password" (which must be at least 8
     characters long) */
  if (generate_Ku(read_session_.securityAuthProto,
                  read_session_.securityAuthProtoLen,
                  reinterpret_cast<u_char *>(const_cast<char *>(snmp_v3_passphrase.c_str())),
                  strlen(snmp_v3_passphrase.c_str()),
                  read_session_.securityAuthKey,
                  &read_session_.securityAuthKeyLen) != SNMPERR_SUCCESS)
  {
    snmp_log(LOG_ERR,
             "Error generating Ku from authentication pass phrase. \n");
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed initialization.");
    return RADIO_SNMP_ERROR;
  }

  /* set the SNMP version number */
  write_session_.version = SNMP_VERSION_3;

  /* set the SNMPv3 user name */
  write_session_.securityName = strdup("userV3");
  write_session_.securityNameLen = strlen(read_session_.securityName);

  /* set the security level to authenticated, but not encrypted */
  write_session_.securityLevel = SNMP_SEC_LEVEL_AUTHNOPRIV;

  /* set the authentication method to MD5 */
  write_session_.securityAuthProto = usmHMACMD5AuthProtocol;
  write_session_.securityAuthProtoLen = sizeof(usmHMACMD5AuthProtocol)/sizeof(oid);
  write_session_.securityAuthKeyLen = USM_AUTH_KU_LEN;

  /* set the authentication key to a MD5 hashed version of our
     passphrase "The Net-SNMP Demo Password" (which must be at least 8
     characters long) */
  if (generate_Ku(write_session_.securityAuthProto,
                  write_session_.securityAuthProtoLen,
                  reinterpret_cast<u_char *>(const_cast<char *>(snmp_v3_passphrase.c_str())),
                  strlen(snmp_v3_passphrase.c_str()),
                  write_session_.securityAuthKey,
                  &write_session_.securityAuthKeyLen) != SNMPERR_SUCCESS)
  {
    snmp_log(LOG_ERR,
             "Error generating Ku from authentication pass phrase. \n");
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed initialization.");
    return RADIO_SNMP_ERROR;
  }

#else /* we'll use the insecure (but simplier) SNMPv1 */

  read_session_.version = SNMP_VERSION_1;
  read_session_.community = const_cast<unsigned char *>(COMMUNITY_READ_STR);
  read_session_.community_len = strlen(reinterpret_cast<const char *>(read_session_.community));

  write_session_.version = SNMP_VERSION_1;
  write_session_.community = const_cast<unsigned char *>(COMMUNITY_WRITE_STR);
  write_session_.community_len = strlen(reinterpret_cast<const char *>(write_session_.community));

#endif

  /* windows32 specific initialization (is a noop on unix) */
  SOCK_STARTUP;

  read_ss_ = snmp_open(&read_session_);
  if (!read_ss_)
  {
    snmp_perror("ack");
    snmp_log(LOG_ERR, "read session error!!!\n");
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed initialization.");
    return RADIO_SNMP_ERROR;
  }

  write_ss_ = snmp_open(&write_session_);
  if (!write_ss_)
  {
    snmp_perror("ack");
    snmp_log(LOG_ERR, "write session error!!!\n");
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed initialization.");
    return RADIO_SNMP_ERROR;
  }

  return RADIO_SNMP_OK;
}

/**
 * @name  DeInit
 * @brief SnmpManager DeInit
 */
int SnmpManager::DeInit()
{
  if (read_ss_)
  {
    snmp_close(read_ss_);
  }
  if (write_ss_)
  {
    snmp_close(write_ss_);
  }

  /* windows32 specific cleanup (is a noop on unix) */
  SOCK_CLEANUP;
}

/**
 * @name  convertInt
 * @brief Convert int to string
 * @param[in] number: integer number to be coverted
 */
string SnmpManager::ConvertIntToString(int64_t number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

/**
 * @name  SnmpGet
 * @brief SNMP Get Function
 * @param[in] param_oid: param OID
 * @param[in, out] param_val: param value
 */
int SnmpManager::SnmpGet(string param_oid, string &param_val)
{
  struct snmp_pdu *pdu;
  struct snmp_pdu *response;
  struct variable_list *vars;
  oid anOID[MAX_OID_LEN];
  size_t anOID_len = MAX_OID_LEN;
  int status;

  pdu = snmp_pdu_create(SNMP_MSG_GET);

  read_objid(param_oid.c_str(), anOID, &anOID_len);
  snmp_add_null_var(pdu, anOID, anOID_len);
  status = snmp_synch_response(read_ss_, pdu, &response);
  if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
  {
# ifdef RADIO_DEBUG
    for (vars = response->variables; vars; vars = vars->next_variable)
       print_variable(vars->name, vars->name_length, vars);
# endif

    for (vars = response->variables; vars; vars = vars->next_variable)
    {
      int count = 1;
      if (vars->type == ASN_OCTET_STR)
      {
        char *sp = static_cast<char *>(malloc(1 + vars->val_len));
        memcpy(sp, vars->val.string, vars->val_len);
        sp[vars->val_len] = '\0';
        // printf("value #%d is a string: %s\n", count++, sp);
        param_val.assign(sp);
        free(sp);
      }
      else if (vars->type == ASN_INTEGER)
      {
        int64_t sp = *(vars->val.integer);
        // printf("value #%d is an integer: %ld\n", count++, sp);
        param_val =  ConvertIntToString(sp);
      }
      else
      {
        /* Corrupted value or empty value read. Possibly caused by
         * old firmware on radio. */
        param_val = "0";
        return RADIO_SNMP_ERROR;
      }
    }
  }
  else
  {
    if (status == STAT_SUCCESS)
    {
      fprintf(stderr, "Error in packet\nReason: %s\n",
              snmp_errstring(response->errstat));
    }
    else
    {
      snmp_sess_perror("snmpget", read_ss_);
    }
    return RADIO_SNMP_ERROR;
  }

  if (response)
  {
    snmp_free_pdu(response);
  }

  return RADIO_SNMP_OK;
}

/**
 * @name  SnmpSet
 * @brief SNMP Set Function
 * @param[in] param_oid: param OID
 * @param[in] param_type: param type, int or string
 * @param[in] param_int_val: int value
 * @param[in] param_str_val: string value
 */
int SnmpManager::SnmpSet(string param_oid, int param_type, int param_int_val, string param_str_val)
{
  struct snmp_pdu *pdu;
  struct snmp_pdu *response;

  oid anOID[MAX_OID_LEN];
  size_t anOID_len = MAX_OID_LEN;
  void *dataptr = NULL;
  size_t datalen = 0;
  int status;

  pdu = snmp_pdu_create(SNMP_MSG_SET);
  read_objid(param_oid.c_str(), anOID, &anOID_len);

  switch (param_type)
  {
    case ASN_INTEGER:
      dataptr = &param_int_val;
      datalen = sizeof(param_int_val);
      break;
    case ASN_OCTET_STR:
      dataptr = const_cast<char *>(param_str_val.c_str());
      datalen = strlen(param_str_val.c_str());
      break;
    default:
      return RADIO_SNMP_ERROR;
  }

  snmp_pdu_add_variable(pdu, anOID, anOID_len, param_type, dataptr, datalen);
  status = snmp_synch_response(write_ss_, pdu, &response);
  if (status != STAT_SUCCESS)
  {
    // printf("SnmpSet error\n");
    return RADIO_SNMP_ERROR;
  }

  if (response)
  {
    snmp_free_pdu(response);
  }

  // printf("SnmpSet ok\n");
  return RADIO_SNMP_OK;
}

/**
 * @name  UpdateStatus
 * @brief Update Radio Status
 * @param[in] status: avidbots_msgs::radio_settings
 */
int SnmpManager::UpdateStatus(avidbots_msgs::radio_settings &status)
{
  string retval;
  string oid;

  // WIFI
  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_SSID_OID;
  SnmpGet(oid, retval);
  // std::cout << "wifi_ssid = " << retval << std::endl;
  status.wifi_ssid = retval;

  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_ENCRYPTION_OID;
  SnmpGet(oid, retval);
  // std::cout << "wifi_authtype = " << retval << std::endl;

  try
  {
    status.wifi_authtype = stoi(retval);
  }
  catch (std::invalid_argument& e)
  {
    // If no conversion could be performed
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed to convert WIFI auth type string (%s) "
              "to int... Exit UpdateStatus..",
              retval.c_str());
    return RADIO_SNMP_ERROR;
  }
  catch (std::out_of_range& e)
  {
    // If the converted value would fall out of the range of the result type
    // or if the underlying function (std::strtol or std::strtoull) sets errno
    // to ERANGE.
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed to convert WIFI auth type string (%s) "
              "to int... Exit UpdateStatus..",
              retval.c_str());
    return RADIO_SNMP_ERROR;
  }
  catch (...)
  {
    // everything else
    ROS_ERROR("AvidbotsRadio:SNMPManager: Failed to convert WIFI auth type string (%s) "
              "to int... Exit UpdateStatus..",
              retval.c_str());
    return RADIO_SNMP_ERROR;
  }

  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_KEY_OID;
  SnmpGet(oid, retval);
  // std::cout << "wifi_password = " << retval << std::endl;
  status.wifi_password = retval;

  oid = SNMP_OID_PREFIX + CARRIER_TMPSTATUS_RSSI_OID;
  SnmpGet(oid, retval);
  // std::cout << "wifi_rssi = " << retval << std::endl;
  status.wifi_rssi = retval;

  // CELL
  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_APN_OID;
  SnmpGet(oid, retval);
  // std::cout << "cell_apn_server = " << retval << std::endl;
  status.cell_apn_server = retval;

  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_USERNAME_OID;
  SnmpGet(oid, retval);
  // std::cout << "cell_apn_user = " << retval << std::endl;
  status.cell_apn_user = retval;

  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_PASSWD_OID;
  SnmpGet(oid, retval);
  // std::cout << "cell_apn_password = " << retval << std::endl;
  status.cell_apn_password = retval;

  oid = SNMP_OID_PREFIX + CARRIER_TMPSTATUS_CONNECT_STATUS_OID;
  SnmpGet(oid, retval);
  // std::cout << "cell_status = " << retval << std::endl;
  status.cell_status = retval;

  oid = SNMP_OID_PREFIX + CARRIER_TMPSTATUS_RSSI_OID;
  SnmpGet(oid, retval);
  // std::cout << "cell_rssi = " << retval << std::endl;
  status.cell_rssi = retval;

  return RADIO_SNMP_OK;
}

int SnmpManager::ApplySettings(const avidbots_msgs::radio_settingsPtr &settings)
{
  int ret;
  string oid;

  // WIFI
  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_SSID_OID;
  ret = SnmpSet(oid, WIFI_VIRTUAL_INTERFACEI_1_SSID_TYPE, 0, settings->wifi_ssid);
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_ENCRYPTION_OID;
  ret = SnmpSet(oid, WIFI_VIRTUAL_INTERFACEI_1_ENCRYPTION_TYPE, settings->wifi_authtype, "");
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  oid = SNMP_OID_PREFIX + WIFI_VIRTUAL_INTERFACEI_1_KEY_OID;
  ret = SnmpSet(oid, WIFI_VIRTUAL_INTERFACEI_1_KEY_TYPE, 0, settings->wifi_password);
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  // CELL
  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_APN_OID;
  ret = SnmpSet(oid, CARRIER_CONNECT1_APN_TYPE, 0, settings->cell_apn_server);
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_USERNAME_OID;
  ret = SnmpSet(oid, CARRIER_CONNECT1_USERNAME_TYPE, 0, settings->cell_apn_user);
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  oid = SNMP_OID_PREFIX + CARRIER_CONNECT1_PASSWD_OID;
  ret = SnmpSet(oid, CARRIER_CONNECT1_PASSWD_TYPE, 0, settings->cell_apn_password);
  if (ret == RADIO_SNMP_ERROR)
  {
    return ret;
  }

  return RADIO_SNMP_OK;
}

