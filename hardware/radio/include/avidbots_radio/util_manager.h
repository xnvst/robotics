/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	util_manager.h
 * @brief	Header file containing the UtilManager class
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_UTIL_MANAGER_H
#define AVIDBOTS_RADIO_UTIL_MANAGER_H

// CPP
#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class UtilManager
{
public:
  UtilManager();
  ~UtilManager();

  bool Ping(std::string ip);

private:
  int Execute(std::string command, std::string &ret);
};

#endif  // AVIDBOTS_RADIO_UTIL_MANAGER_H
