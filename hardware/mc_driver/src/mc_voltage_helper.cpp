/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	mc_voltage_helper.cpp
 * @brief	Source File containing the MC Voltage Helper class
 * @author  Tony Lu
 */

// LOCAL
#include "avidbots_mc_driver/mc_voltage_helper.h"

// CPP
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <cmath>

/**
 * @name    VoltageCalculations
 * @brief   Main voltage averaging function.
 * @param[in] data: Buffer of voltage data.
 * @param[in] recent_voltage: Latest voltage of mc driver
 */
double mc_voltage_helper::GetAverageVoltage(const boost::circular_buffer<int> &data, double recent_voltage)
{
  double mode_batt_voltage;

  if (data.size() > 0)
    mode_batt_voltage = GetMode(data);
  else
    mode_batt_voltage = recent_voltage;

  return mode_batt_voltage;
}

 /**
 * @name    GetMode
 * @brief   Returns the mode of a circular buffer
 * @return  Returns zero if the data size is zero
 * @param[in] data: Buffer of voltage data.
 */
double mc_voltage_helper::GetMode(const boost::circular_buffer<int> &data)
{
  int mode = 0;
  int max_frequency = 0;
  double u = Mean(data);
  double s = StandardDeviation(data, u);
  int m = 1;  // Number of standard deviations
  double lower = u - (m * s);
  double upper = u + (m * s);

  std::unordered_map<int, int> frequency;
  boost::circular_buffer<int>::const_iterator buff_it;
  std::unordered_map<int, int>::iterator map_it;

  /* Inserts frequency of each data value into hash table */
  for (buff_it = data.begin(); buff_it != data.end(); ++buff_it)
  {
    /* Accepts elements that are within m standard deviations */
    if (*buff_it >= lower && *buff_it <= upper)
    {
      map_it = frequency.find(*buff_it);
      if (map_it != frequency.end())
        map_it->second += 1;
      else
        frequency.insert(std::pair<int, int>(*buff_it, 1));
    }
  }

  /* Gets the most frequent data value */
  for (map_it = frequency.begin(); map_it != frequency.end(); ++map_it)
  {
    if (map_it->second > max_frequency)
    {
      max_frequency = map_it->second;
      mode = map_it->first;
    }
  }

  return mode;
}

/**
 * @name    Mean
 * @brief   Calculates the mean of a collection of numbers
 * @param[in] data: Vector of voltage data.
 */
double mc_voltage_helper::Mean(const boost::circular_buffer<int> &data)
{
  double mean = 0;
  double sum = 0;
  boost::circular_buffer<int>::const_iterator it;

  if (data.size() > 0)
  {
    for (it = data.begin(); it != data.end(); ++it)
    {
      sum += *it;
    }
    mean = sum / data.size();
  }

  return mean;
}

/**
 * @name    Standard Devation
 * @brief   Calculates standard deviation of a collection of numbers
 * @param[in] data: The incoming data.
 * @param[in] mean: The average of the data values.
 */
double mc_voltage_helper::StandardDeviation(const boost::circular_buffer<int> &data, double mean)
{
  double sd = 0;
  double temp = 0;
  boost::circular_buffer<int>::const_iterator it;

  if (data.size() > 0)
  {
    for (it = data.begin(); it != data.end(); ++it)
    {
      temp += (*it - mean) * (*it - mean);
    }
    sd = sqrt(temp / data.size());
  }

  return sd;
}

/**
 * @name    Slope
 * @brief   Calculates the slope of two cartesian points
 * @param[in] x and y coordinates of two points
 */
double mc_voltage_helper::Slope(double x1, double y1, double x2, double y2)
{
  return (y2-y1)/(x2-x1);
}

/**
 * @name    YIntercept
 * @brief   Calculates the the y intercept given slope and a point
 * @param[in] slope and x and y coordinates of a point
 */
double mc_voltage_helper::YIntercept(double slope, double x, double y)
{
  return y-slope*x;
}
