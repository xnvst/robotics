/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2014, Avidbots Corp.
 * @name	Test CAN Open
 * @brief	This application is designed to run a basic speed test
          a certain roboteq CAN device.
 * @author	Pablo Molina
 */

#include "avidbots_mc_driver/roboteq_nxtgen_can.h"
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

//////////
// MAIN //
//////////

int main()
{
  RoboteqNxtGenCANOpenController motor_controller_1;

  motor_controller_1.InitPort("can0", 100);

  int count = 1;

  int8_t temp;
  int16_t amps;
  int16_t rpm;
  int32_t ticks;

  int command_return = 0;
  command_return = motor_controller_1.GetMCTemp(0x0A, temp);
  usleep(1000*100);

  printf("The return value is: %d , the temp is: %d \n", command_return,
           temp);


  command_return = motor_controller_1.GetMotorAmps(0x0A, amps);
  usleep(1000*100);

  printf("The return value is: %d , the current is: %d \n", command_return,
           amps);



  while (1)
  {
    count = count +1;

    usleep(1000*5);
    command_return = motor_controller_1.GetBrushlessTicks(0x0A, ticks);
    printf("The return value is: %d , the brushless ticks is: %d \n",
            command_return, ticks);


    usleep(1000*5);
    command_return = motor_controller_1.GetEncoderRPM(0x0A, rpm);
    printf("The return value is: %d , the RPM is: %d \n",
            command_return, rpm);

    usleep(1000*5);
    printf("The current command value is: %d \n" , 50*count);
    command_return = motor_controller_1.SetMotorSpeed(0x0A, 50*count);
    printf("The return value is: %d, done setting the speed \n",
            command_return);


    usleep(1000*1000);

    if (count >= 20 )
    {
      count = 0;
    }
  }
}
