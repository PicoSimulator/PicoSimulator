#include "platform/rpi/rp2040/pad.hpp"
#include "platform/rpi/rp2040/gpio.hpp"

#include <iostream>

using namespace RP2040;
void Pad::net_state_changed(){
  // m_input_value_raw = connected_net()->digital_read();
}

void Pad::pad_state_changed(){
  connected_net()->update();
}


void Pad::update()
{
  // std::cerr << "Pad::update(" << this << ")" << std::endl;
  // std::cerr << "  gpio oe: " << m_gpio.get_output_enable() << std::endl;
  if (m_gpio.get_output_enable()) {
    int drive_strengths[] = {96, 86, 76, 70};
    NetConnection::set_drive_strength(drive_strengths[(int)get_drive_strength()]);
    // std::cerr << "  gpio out: " << m_gpio.get_output() << std::endl;
    set_drive_value(m_gpio.get_output());
    if (is_connected()) {
      connected_net()->update();
    } else {
      // std::cerr << "Pad::update(" << this << ") not connected" << std::endl;
    }
  }
}