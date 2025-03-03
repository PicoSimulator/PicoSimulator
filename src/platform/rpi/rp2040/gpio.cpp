#include "platform/rpi/rp2040/gpio.hpp"
#include "platform/rpi/rp2040/pad.hpp"

#include <iostream>

using namespace RP2040;

#define NAME (m_pad.is_connected() ? m_pad.connected_net()->name() : "NC")

void RP2040::GPIO::update_from_internal(){
  m_pad.update();
}

void RP2040::GPIO::set_funcsel(uint8_t funcsel){
  current_signal().gpio() = nullptr;
  m_funcsel = funcsel;
  current_signal().gpio() = this;
  std::cerr << "RP2040::GPIO::set_funcsel(" << this << ", " << (int)funcsel << ")" << std::endl;
  update_from_internal();
}

void RP2040::GPIO::set_oe_override(Override oe){
  m_oe_override = oe;
  std::cerr << "RP2040::GPIO::set_oe_override(" << this << ", " << oe << ")" << std::endl;
  update_from_internal();
}

void RP2040::GPIO::set_output_override(Override out){
  m_output_override = out;
  std::cerr << "RP2040::GPIO::set_output_override(" << this << ", " << out << ")" << std::endl;
  update_from_internal();
}

void RP2040::GPIO::set_input_override(Override in){
  m_input_override = in;
  update_from_internal();
}

void RP2040::GPIO::set_irq_override(Override irq){
  m_irq_override = irq;
  update_from_internal();
}

void RP2040::GPIO::set_ctrl(uint32_t ctrl){
  uint8_t funcsel = ctrl & 0x1f;
  RP2040::GPIO::Override outover = static_cast<RP2040::GPIO::Override>((ctrl >> 8) & 0x3);
  RP2040::GPIO::Override oeover = static_cast<RP2040::GPIO::Override>((ctrl >> 12) & 0x3);
  RP2040::GPIO::Override inover = static_cast<RP2040::GPIO::Override>((ctrl >> 16) & 0x3);
  RP2040::GPIO::Override irqover = static_cast<RP2040::GPIO::Override>((ctrl >> 28) & 0x3);
  current_signal().gpio() = nullptr;
  m_funcsel = funcsel;
  current_signal().gpio() = this;
  std::cerr << "RP2040::GPIO::set_funcsel(" << this << ", " << (int)funcsel << ") " << NAME << std::endl;
  m_output_override = outover;
  m_oe_override = oeover;
  m_input_override = inover;
  m_irq_override = irqover;
  update_from_internal();
}

uint32_t RP2040::GPIO::get_ctrl() const{
  return m_funcsel | (m_output_override << 8) | (m_oe_override << 12) | (m_input_override << 16) | (m_irq_override << 28);
}

uint32_t RP2040::GPIO::get_status() const{
  return 
      current_signal().get_output() << 8
    | get_output() << 9
    | current_signal().get_oe() << 12
    | get_output_enable() << 13
    | 0 /* INFROMPAD */ << 17
    | 0 /* INTOPERI */ << 19
    | 0 /* IRQFROMPAD */ << 24
    | 0 /* IRQTOPROC */ << 26;
}

