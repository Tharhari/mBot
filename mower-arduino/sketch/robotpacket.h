#ifndef ROBOTPACKET_H
#define ROBOTPACKET_H

#include "robotcommand.h"

#include <stdint.h>

template <typename T>
class RobotPacket {
  private:
    RobotCommand command;
    T parameter;
    T from_bytes(int8_t *buffer);
    
  public:
    RobotPacket(RobotCommand command, T parameter);
    RobotPacket(RobotCommand command, int8_t* buffer);
    RobotCommand get_command();
    T get_parameter();
    void to_bytes(int8_t* buffer);
    int32_t get_length();
};

#endif
