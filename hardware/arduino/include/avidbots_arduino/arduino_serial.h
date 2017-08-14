/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 * @name arduino_serial.h
 * @brief Header file that defines the arduino serial class
 * @author Pablo Molina
 */

#ifndef AVIDBOTS_ARDUINO_ARDUINO_SERIAL_H
#define AVIDBOTS_ARDUINO_ARDUINO_SERIAL_H

#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int64.h>

#include <stdint.h>



class ArduinoSerial
{
public:
  explicit ArduinoSerial(ros::NodeHandle* ptr);

  int Init(const ros::NodeHandle& private_nh);

  int ReadLoop();

private:
  void setupSerialPort();

  uint8_t ComputeCRC8(const uint8_t *addr, uint8_t len);

  int fd;

  std::string port_name;

  // ROS related

  ros::Publisher    free_wheel_enc_pub_;
  ros::NodeHandle*  node_handle_;
};


#endif  // AVIDBOTS_ARDUINO_ARDUINO_SERIAL_H
