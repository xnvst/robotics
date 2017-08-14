/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	mc_voltage_helper.h
 * @brief	Header File containing the MC Voltage Helper class
 * @author	Tony Lu
 */

#ifndef AVIDBOTS_MC_DRIVER_MC_VOLTAGE_HELPER_H
#define AVIDBOTS_MC_DRIVER_MC_VOLTAGE_HELPER_H

#include <boost/circular_buffer.hpp>

namespace mc_voltage_helper
{
double GetAverageVoltage(const boost::circular_buffer<int> &data, double recent_voltage);
double GetMode(const boost::circular_buffer<int> &data);
double Mean(const boost::circular_buffer<int> &data);
double StandardDeviation(const boost::circular_buffer<int> &data, double mean);
double Slope(double x1, double y1, double x2, double y2);
double YIntercept(double slope, double x, double y);
}

#endif  // AVIDBOTS_MC_DRIVER_MC_VOLTAGE_HELPER_H
