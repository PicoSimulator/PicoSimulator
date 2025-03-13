#include "usb/host.hpp"
#include "usb/device.hpp"

using namespace USB;

void Host::receive_packet

void Host::send_sof(uint16_t frame_num)
{
  frame_num &= 0x7FF;
  uint8_t buf[3];
  buf[0] = reinterpret_cast<uint8_t>(PID::SOF);
  buf[1] = frame_num & 0xff;
  buf[2] = (frame_num>>8);
  
}