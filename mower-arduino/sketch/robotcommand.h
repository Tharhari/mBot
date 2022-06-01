#ifndef ROBOTCOMMAND_H
#define ROBOTCOMMAND_H

#include <stdint.h>

enum RobotCommand : int8_t
{
  CONNECTED = 0,
  DISCONNECTED = 1,
  ACCELERATION = 2,
  STEERING = 3,
  COLLISION = 4,
  MODE = 5
};
  

#endif
