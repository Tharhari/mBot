#include "robotpacket.h"

template <typename T>
RobotPacket<T>::RobotPacket(RobotCommand command, T parameter) 
  : command(command), parameter(parameter) {}

template <typename T>
RobotPacket<T>::RobotPacket(RobotCommand command, int8_t buffer[]) : command(command) 
{
  this->parameter = this->from_bytes(buffer);
}
 
template <typename T>   
RobotCommand RobotPacket<T>::get_command() { return this->command; }

template <typename T> 
T RobotPacket<T>::get_parameter() { return this->parameter; }

template <typename T>
int32_t RobotPacket<T>::get_length() { return sizeof(this->command) + sizeof(this->parameter); }


template<>
int8_t RobotPacket<int8_t>::from_bytes(int8_t *buffer) {
  return (static_cast<int8_t>(*(buffer + 0)));
}

template<> 
int16_t RobotPacket<int16_t>::from_bytes(int8_t *buffer) {
  return (static_cast<int16_t>(*(buffer + 0)) & 0xff) | 
         (static_cast<int16_t>(*(buffer + 1)) << 8 & 0xff00);
}

template<> 
int32_t RobotPacket<int32_t>::from_bytes(int8_t *buffer) {
  return (static_cast<int32_t>(*(buffer + 0))       & 0xff)     |
         (static_cast<int32_t>(*(buffer + 1)) << 8  & 0xff00)   | 
         (static_cast<int32_t>(*(buffer + 2)) << 16 & 0xff0000) | 
         (static_cast<int32_t>(*(buffer + 3)) << 24 & 0xff000000);
}

template<> 
void RobotPacket<int8_t>::to_bytes(int8_t *buffer) {
  buffer[0] = this->command & 0xff;
  buffer[1] = this->parameter;
}

template<> 
void RobotPacket<int16_t>::to_bytes(int8_t *buffer) {
  buffer[0] = this->command & 0xff;
  buffer[1] = this->parameter & 0xff;
  buffer[2] = this->parameter >> 8;
}

template<> 
void RobotPacket<int32_t>::to_bytes(int8_t *buffer) {
  buffer[0] = this->command & 0xff;
  buffer[1] = this->parameter & 0xff;
  buffer[2] = this->parameter >> 8;
  buffer[3] = this->parameter >> 16;
  buffer[4] = this->parameter >> 24;
}

template class RobotPacket<int8_t>;
template class RobotPacket<int16_t>;
template class RobotPacket<int32_t>;
