/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	telnet_manager.cpp
 * @brief	Source File containing the TelnetManager class
 * @author	Feng Cao
 */

// ROS
#include <ros/ros.h>

// CPP
#include <string>

// LOCAL
#include "avidbots_radio/telnet_manager.h"
#include "avidbots_radio/util_manager.h"


struct termios TelnetManager::tin_;
struct termios TelnetManager::tlocal_;

/**
 * @name 	TelnetManager
 * @brief	Default constructor
 */
TelnetManager::TelnetManager(RadioManager* radio_mgr) : radio_mgr_(radio_mgr)
{
  telnet_init_flag_ = 0;
  recv_buffer_pos_ = 0;
}

/**
 * @name  ~TelnetManager
 * @brief Destructor
 */
TelnetManager::~TelnetManager() {}

/**
 * @name  Init
 * @brief TelnetManager Init
 */
int TelnetManager::Init()
{
  if (telnet_init_flag_)
  {
    return 0;
  }

  // Create socket
  sock_ = socket(AF_INET , SOCK_STREAM , 0);
  if (sock_ == -1)
  {
    perror("Could not create socket. Error");
    return 1;
  }

  server_.sin_addr.s_addr = inet_addr(RADIO_AGENT_IP.c_str());
  server_.sin_family = AF_INET;
  server_.sin_port = htons(telnet_constants::PORT);

  // Connect to remote server
  if (connect(sock_, (struct sockaddr *)&server_, sizeof(server_)) < 0)
  {
    perror("connect failed. Error");
    return 1;
  }
  // puts("Connected...\n");

  // set terminal
  SetTerminal();
  atexit(ResetTerminal);

  ts_.tv_sec = 1;
  ts_.tv_usec = 0;

  telnet_init_flag_ = 1;
}

/**
 * @name  DeInit
 * @brief TelnetManager DeInit
 */
int TelnetManager::DeInit()
{
  close(sock_);
  telnet_init_flag_ = 0;
}

/**
 * @name  Negotiate
 * @brief TelnetManager Negotiate
 */
int TelnetManager::Negotiate(int socket, unsigned char *buf, int len)
{
  int i;

  if (buf[1] == telnet_constants::DO && buf[2] == telnet_constants::CMD_WINDOW_SIZE)
  {
    unsigned char tmp1[10] = {255, 251, 31};
    if (send(socket, tmp1, 3 , 0) < 0)
    {
      return 1;
    }

    unsigned char tmp2[10] = {255, 250, 31, 0, 80, 0, 24, 255, 240};
    if (send(socket, tmp2, 9, 0) < 0)
    {
      return 1;
    }
    return 0;
  }

  for (i = 0; i < len; i++)
  {
    if (buf[i] == telnet_constants::DO)
    {
      buf[i] = telnet_constants::WONT;
    }
    else if (buf[i] == telnet_constants::WILL)
    {
      buf[i] = telnet_constants::DO;
    }
  }

  if (send(socket, buf, len , 0) < 0)
  {
    return 1;
  }
}

/**
 * @name  SetTerminal
 * @brief TelnetManager Set Terminal
 */
void TelnetManager::SetTerminal(void)
{
  // save terminal configuration
  tcgetattr(STDIN_FILENO, &tin_);

  memcpy(&tlocal_, &tin_, sizeof(tin_));
  cfmakeraw(&tlocal_);
  tcsetattr(STDIN_FILENO, TCSANOW, &tlocal_);
}

/**
 * @name  ResetTerminal
 * @brief TelnetManager Reset Terminal
 */
void TelnetManager::ResetTerminal(void)
{
  // restore terminal upon exit
  tcsetattr(STDIN_FILENO, TCSANOW, &tin_);
}

/**
 * @name  TelnetLogin
 * @brief TelnetManager Telnet Login
 */
int TelnetManager::TelnetLogin()
{
  memset(send_buffer_, 0, sizeof(send_buffer_));
  send_buffer_[0] = 'a';
  send_buffer_[1] = 'd';
  send_buffer_[2] = 'm';
  send_buffer_[3] = 'i';
  send_buffer_[4] = 'n';
  send_buffer_[5] = '\n';
  if (send(sock_, send_buffer_, 6, 0) < 0)
  {
    return 1;
  }
  recv_buffer_pos_ = 0;
  memset(recv_buffer_, 0, sizeof(recv_buffer_));
  return 0;
}

/**
 * @name  TelnetPassword
 * @brief TelnetManager Telnet Password
 */
int TelnetManager::TelnetPassword()
{
  memset(send_buffer_, 0, sizeof(send_buffer_));
  int pass_len = static_cast<int>(strlen(snmp_v3_passphrase.c_str()));
  memcpy(send_buffer_, snmp_v3_passphrase.c_str(), pass_len);
  send_buffer_[pass_len] = '\n';
  if (send(sock_, send_buffer_, pass_len+1, 0) < 0)
  {
    return 1;
  }
  recv_buffer_pos_ = 0;
  memset(recv_buffer_, 0, sizeof(recv_buffer_));
  return 0;
}

/**
 * @name  TelnetInfo
 * @brief TelnetManager Telnet Info
 */
int TelnetManager::TelnetInfo()
{
  memset(send_buffer_, 0, sizeof(send_buffer_));
  send_buffer_[0] = 's';
  send_buffer_[1] = 't';
  send_buffer_[2] = 'a';
  send_buffer_[3] = 't';
  send_buffer_[4] = 'u';
  send_buffer_[5] = 's';
  send_buffer_[6] = ' ';
  send_buffer_[7] = 'n';
  send_buffer_[8] = 'e';
  send_buffer_[9] = 't';
  send_buffer_[10] = 'w';
  send_buffer_[11] = 'o';
  send_buffer_[12] = 'r';
  send_buffer_[13] = 'k';
  send_buffer_[14] = '\n';
  if (send(sock_, send_buffer_, 15, 0) < 0)
  {
    return 1;
  }
  recv_buffer_pos_ = 0;
  memset(recv_buffer_, 0, sizeof(recv_buffer_));
  return 0;
}

/**
 * @name  TelnetATW
 * @brief TelnetManager Telnet ATW
 */
int TelnetManager::TelnetATW()
{
  memset(send_buffer_, 0, sizeof(send_buffer_));
  send_buffer_[0] = 'A';
  send_buffer_[1] = 'T';
  send_buffer_[2] = '&';
  send_buffer_[3] = 'W';
  send_buffer_[4] = '\n';
  if (send(sock_, send_buffer_, 5, 0) < 0)
  {
    return 1;
  }
  recv_buffer_pos_ = 0;
  memset(recv_buffer_, 0, sizeof(recv_buffer_));
  return 0;
}

/**
 * @name  TelnetATO
 * @brief TelnetManager Telnet ATO
 */
int TelnetManager::TelnetATO()
{
  memset(send_buffer_, 0, sizeof(send_buffer_));
  send_buffer_[0] = 'A';
  send_buffer_[1] = 'T';
  send_buffer_[2] = 'O';
  send_buffer_[3] = '\n';
  if (send(sock_, send_buffer_, 4, 0) < 0)
  {
    return 1;
  }
  recv_buffer_pos_ = 0;
  memset(recv_buffer_, 0, sizeof(recv_buffer_));
  return 0;
}

/**
 * @name  UpdateStatus
 * @brief Update Radio Status
 * @param[in] status: avidbots_msgs::radio_settings
 */
int TelnetManager::UpdateStatus(avidbots_msgs::radio_settings &status)
{
  char * pch;
  pch = strstr(reinterpret_cast<char *>(wan_status_), "WAN Interface");
  if (pch)
  {
    status.wan_status.assign(pch, strlen(pch) - 12);
  }
}

/**
 * @name  TelnetProc
 * @brief TelnetManager Telnet Process
 */
int TelnetManager::TelnetProc(telnet_constants::TELNET_PROC_CODE code)
{
  unsigned char buf[16];
  int len;
  int buffer_state = 0;

  int exit_flag = 0;

  if (Init())
  {
    return 1;
  }

  while (!exit_flag)
  {
    // select setup
    fd_set fds;
    FD_ZERO(&fds);
    if (sock_ != 0)
    FD_SET(sock_, &fds);
    FD_SET(0, &fds);
    // wait for data
    int nready = select(sock_ + 1, &fds, reinterpret_cast<fd_set *>(0), reinterpret_cast<fd_set *>(0), &ts_);
    if (nready < 0)
    {
      perror("select. Error");
      DeInit();
      return 1;
    }
    else if (nready == 0) {
      ts_.tv_sec = 1;
      ts_.tv_usec = 0;
    }
    else if (sock_ != 0 && FD_ISSET(sock_, &fds))
    {
      // start by reading a single byte
      int rv;
      if ((rv = recv(sock_, buf, 1, 0)) < 0)
      {
        DeInit();
        return 1;
      }
      else if (rv == 0)
      {
        printf("1:Connection closed by the remote end\n\r");
        TelnetATO();
        DeInit();
        return 0;
      }
      if (buf[0] == telnet_constants::CMD)
      {
        // read 2 more bytes
        len = recv(sock_, buf + 1, 2, 0);
        if (len  < 0)
          return 1;
        else if (len == 0) {
          printf("2:Connection closed by the remote end\n\r");
          TelnetATO();
          DeInit();
          return 0;
        }
        Negotiate(sock_, buf, 3);
      }
      else
      {
        len = 1;
        buf[len] = '\0';
        recv_buffer_[recv_buffer_pos_++] = buf[0];
        // printf("%s", buf);
        fflush(0);
        if (strstr(reinterpret_cast<char *>(recv_buffer_), "login:"))
        {
          TelnetLogin();
        }
        else if (strstr(reinterpret_cast<char *>(recv_buffer_), "Password:"))
        {
          TelnetPassword();
        }
        else if (strstr(reinterpret_cast<char *>(recv_buffer_), ">"))
        {
          switch (buffer_state)
          {
            case 0:
              if (code == telnet_constants::telnet_proc_status)
              {
                TelnetInfo();
              }
              else if (code == telnet_constants::telnet_proc_reset)
              {
                TelnetATW();
              }
              buffer_state = 1;
              break;
            case 1:
              if (code == telnet_constants::telnet_proc_status)
              {
                memset(wan_status_, 0, sizeof(wan_status_));
                memcpy(wan_status_, recv_buffer_, sizeof(wan_status_));
              }
              else if (code == telnet_constants::telnet_proc_reset)
              {
                TelnetATO();
              }
              buffer_state = 2;
              break;
            case 2:
              TelnetATO();
              exit_flag = 1;
              break;
          }
        }
      }
    }
  }

  DeInit();
  return 0;
}
