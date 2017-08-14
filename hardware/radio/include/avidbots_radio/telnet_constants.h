/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2017, Avidbots Corp.
 * @name	Telnet Constants header file
 * @brief	Telnet Constants
 * @author	Feng Cao
 */

#ifndef AVIDBOTS_RADIO_TELNET_CONSTANTS_H
#define AVIDBOTS_RADIO_TELNET_CONSTANTS_H

namespace telnet_constants
{
  enum TELNET_ERROR_CODE
  {
    radio_telnet_ok = 0,
    radio_telnet_error = 1,
  };

  enum TELNET_PROC_CODE
  {
    telnet_proc_status = 0,
    telnet_proc_reset = 1,
  };

  const int PORT = 23;
  const int DO = 0xfd;
  const int WONT = 0xfc;
  const int WILL = 0xfb;
  const int DONT = 0xfe;
  const int CMD = 0xff;
  const int CMD_ECHO = 1;
  const int CMD_WINDOW_SIZE = 31;
}  // namespace telnet_constants

#endif  // AVIDBOTS_RADIO_TELNET_CONSTANTS_H

