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
 * @brief	Source File containing the UtilManager class
 * @author	Feng Cao
 */

// ROS
#include <ros/ros.h>

// CPP
#include <string>

// LOCAL
#include "avidbots_radio/util_manager.h"

/**
 * @name 	UtilManager
 * @brief	Default constructor
 */
UtilManager::UtilManager()
{
}

/**
 * @name  ~UtilManager
 * @brief Destructor
 */
UtilManager::~UtilManager() {}

/**
 * @name  Execute
 * @brief Execute input command string
 * @param[in] command: command string to be executed
 * @param[in, out] ret: value returned from execution
 */
int UtilManager::Execute(std::string command, std::string &ret)
{
  FILE *fp;
  char result[1024];

  fp = popen(command.c_str(), "r");
  if (fp == NULL)
  {
    printf("Failed to run command\n");
    return -1;
  }

  /* Read the output a line at a time - output it. */
  while (fgets(result, sizeof(result)-1, fp) != NULL)
  {
    ROS_DEBUG("%s\n", result);
  }

  /* close */
  pclose(fp);

  ret = std::string(result);

  return 0;
}

/**
 * @name  Ping
 * @brief implement ping function
 */
bool UtilManager::Ping(std::string ip)
{
  bool flag = false;
  std::string ret;
  std::string cmd = "ping -c 3 " + ip + " | grep -c ms";
  Execute(cmd, ret);
  if (strcmp(ret.c_str(), "5\n") == 0)
  {
    // printf("agent host is alive\n");
    flag = true;
  }
  else
  {
    // printf("agent host is not alive\n");
  }
  return flag;
}
