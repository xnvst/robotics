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
 * @brief	Header file containing the TelnetManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_TELNET_MANAGER_H
#define AVIDBOTS_RADIO_TELNET_MANAGER_H

// CPP
#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>

// LCOAL
#include "avidbots_radio/telnet_constants.h"
#include "avidbots_radio/radio_manager.h"

class RadioManager;

class TelnetManager
{
public:
  explicit TelnetManager(RadioManager* radio_mgr);
  ~TelnetManager();

  int Init();
  int DeInit();

  int TelnetProc(telnet_constants::TELNET_PROC_CODE code);
  int UpdateStatus(avidbots_msgs::radio_settings &status);

private:
  RadioManager* radio_mgr_;

  unsigned char wan_status_[1024];

  int telnet_init_flag_;
  int sock_;
  struct sockaddr_in server_;
  struct timeval ts_;

  unsigned char recv_buffer_[1024];
  unsigned char send_buffer_[64];
  int recv_buffer_pos_;

  static struct termios tin_;
  static struct termios tlocal_;

  static void ResetTerminal(void);

  int Negotiate(int sock, unsigned char *buf, int len);
  void SetTerminal(void);

  int TelnetLogin();
  int TelnetPassword();
  int TelnetInfo();
  int TelnetATW();
  int TelnetATO();
};

#endif  // AVIDBOTS_RADIO_TELNET_MANAGER_H
