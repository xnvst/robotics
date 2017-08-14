/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2016, Avidbots Corp.
 * @name    state_base
 * @brief   Implementation file for StateBase class
 * @author  Keshav Iyengar, Feng Cao
 */

// CPP
#include <string>

// ROS
#include <ros/ros.h>

// LOCAL
#include "avidbots_safety_monitor/states/state_base.h"

/**
 * @name        StateBase
 * @brief       Constructor.
 * @param[in]   manager: manager object
 */
StateBase::StateBase(SMManager* manager): manager_(manager)
{
}

