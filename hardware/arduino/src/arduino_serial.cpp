/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015, Avidbots Corp.
 * @name arduino_serial.cpp
 * @brief Source file that defines the arduino serial class
 * @author Pablo Molina
 */


#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <iostream>
#include <stdint.h>

// ROS related
#include <avidbots_msgs/topics.h>
#include <avidbots_library/get_param/get_param_util.h>
#include "avidbots_arduino/arduino_serial.h"

/**
 * @name  ArduinoSerial
 * @brief Constructor
 */
ArduinoSerial::ArduinoSerial(ros::NodeHandle* ptr) : node_handle_(ptr) { }


/**
 * @name  Init
 * @brief Initialize class
 */
int ArduinoSerial::Init(const ros::NodeHandle &private_nh)
{
  GetParamUtil::GetParam("/avidbots_arduino_serial/arduino_port", port_name, "/dev/avidbots/arduino");

  fd = 0;

  while (true)
  {
    // Open file descriptor
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);

    // Check to see if fd was opened successfully
    if (fd > 0)
    {
      break;
    }
    else
    {
      ROS_INFO_STREAM("Unable to open serial port: " << port_name << ". Error: " <<
                      fd << ". Will retry every second");
      sleep(1);
    }
  }

  // Setup serial port
  setupSerialPort();

  free_wheel_enc_pub_ = node_handle_->advertise
                        <std_msgs::Int64>
                        (avidbots_topics::free_wheel_topic, 1, false);

  return fd;
}

/**
 * @name  setupSerialPort
 * @brief Setups Serial port using default settings from arduino serial library
 */
void ArduinoSerial::setupSerialPort()
{
  /* Set up the control structure */
  struct termios toptions;

  /* Get currently set options for the tty */
  tcgetattr(fd, &toptions);

  /* Set custom options */

  /* 9600 baud */
  cfsetispeed(&toptions, B9600);
  cfsetospeed(&toptions, B9600);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;
  /* wait for 1 characters to come in before read returns */
  toptions.c_cc[VMIN] = 1;
  /* no minimum time to wait before read returns */
  toptions.c_cc[VTIME] = 0;
  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);
  /* Wait for the Arduino to reset */
  sleep(1);
  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);
}

/**
 * @name  ReadLoop
 * @brief Main read loop. This function does not return
 */
int ArduinoSerial::ReadLoop()
{
  char buf[1];
  char buffer[256];
  int encoder_val       = 0;
  int crc8_val          = 0;
  uint8_t crc8_computed = 0;

  while (1)
  {
    //  Read incoming messages
    int len = 0;
    memset(buffer, 0, 64);
    try
    {
      while (1)
      {
        int n = read(fd, buf, 1);
        if (buf[0] == '\n')
        {
          //  printf("%s", buffer);
          buffer[len] = 0;
          break;
        }
        else
        {
          //  printf("%c", buf[0]);
          buffer[len] = buf[0];
          len++;
          if (len > 64)
          {
            ROS_ERROR_STREAM("Unable to find endline, skipping this message");
            break;
          }
        }
      }
    }
    catch (int e)
    {
      ROS_ERROR_STREAM("Error while reading serial port: " << e << " exiting.");
      return -1;
    }


    // Tokenizing incoming string
    try
    {
      char * pch;
      pch = strtok(buffer, "#");  //  NOLINT
      encoder_val = atoi(pch);
      //  printf ("Value for encoder is: %i\n",encoder_val);
      pch = strtok(NULL, "#");  //  NOLINT
      crc8_val = atoi(pch);
      //  printf ("Value for CRC8 is: %i\n",crc8_val);

      // Checking CRC8
      uint8_t encoder_array [4];
      encoder_array[0]  = (encoder_val) & 0xff;
      encoder_array[1]  = (encoder_val >> (8)) & 0xff;
      encoder_array[2]  = (encoder_val >> (16)) & 0xff;
      encoder_array[3]  = (encoder_val >> (24)) & 0xff;
      crc8_computed     = ComputeCRC8(encoder_array, 4);
      // printf("The computed CRC8 is: %i\n", crc8_computed);

      // Publishing data
      if (crc8_computed == crc8_val)
      {
        std_msgs::Int64 msg_out;
        msg_out.data = encoder_val;
        free_wheel_enc_pub_.publish(msg_out);
      }
      else
      {
        ROS_ERROR_STREAM("CRC8 does not match. Skipping this message");
      }
    }
    catch (int e)
    {
      ROS_ERROR_STREAM("Failed to tokenize/compute CRC8" << e << ". Skipping this message");
    }

    ros::spinOnce();
  }
}

/**
 * @name  ComputeCRC8
 * @brief Computes the CRC8 according to CRC-8-Dallas/Maxim polynomial
 * @param[in] addr: The address to the array containing the number
 * @param[in] addr: The lenght of the array
 * @return : The CRC8 value
 */
uint8_t ArduinoSerial::ComputeCRC8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--)
  {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}
