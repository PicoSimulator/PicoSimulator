#include "rp2040/gpio.hpp"
#include "rp2040/pad.hpp"

#include <iostream>

using namespace RP2040;


GPIOSignal &GPIOSignal::dummy(){
  static GPIOSignal dummy;
  return dummy;
}

void GPIOSignal::set_output(bool val){
  m_output_value = val;
  // std::cerr << "GPIO::set_output(" << this << ", " << val << ")" << std::endl;
  if (m_gpio)
    m_gpio->update_from_mux();
  // else
  //   std::cerr << "  no gpio" << std::endl;
}

void GPIOSignal::set_oe(bool oe){
  m_output_enable = oe;
  // std::cerr << "GPIO::set_oe(" << this << ", " << oe << ")" << std::endl;
  if (m_gpio)
    m_gpio->update_from_mux();
}

bool GPIOSignal::get_input() const{
  if (m_input_gpio)
    return m_input_gpio->get_input();
  else if(m_gpio)
    return m_gpio->get_input();
  return false;
}

void GPIO::update_from_mux(){
  m_pad.update();
}

void GPIO::set_funcsel(uint8_t funcsel){
  current_signal().m_gpio = nullptr;
  m_funcsel = funcsel;
  current_signal().m_gpio = this;
  // std::cerr << "GPIO::set_funcsel(" << this << ", " << (int)funcsel << ")" << std::endl;
  update_from_mux();
}

void GPIO::set_oe_override(Override oe){
  m_oe_override = oe;
  // std::cerr << "GPIO::set_oe_override(" << this << ", " << oe << ")" << std::endl;
  update_from_mux();
}

void GPIO::set_output_override(Override out){
  m_output_override = out;
  // std::cerr << "GPIO::set_output_override(" << this << ", " << out << ")" << std::endl;
  update_from_mux();
}

void GPIO::set_input_override(Override in){
  m_input_override = in;
  // update_from_mux();
}

void GPIO::set_irq_override(Override irq){
  m_irq_override = irq;
  // update_from_mux();
}

void GPIO::set_ctrl(uint32_t ctrl){
  uint8_t funcsel = ctrl & 0x1f;
  GPIO::Override outover = static_cast<GPIO::Override>((ctrl >> 8) & 0x3);
  GPIO::Override oeover = static_cast<GPIO::Override>((ctrl >> 12) & 0x3);
  GPIO::Override inover = static_cast<GPIO::Override>((ctrl >> 16) & 0x3);
  GPIO::Override irqover = static_cast<GPIO::Override>((ctrl >> 28) & 0x3);
  current_signal().m_gpio = nullptr;
  m_funcsel = funcsel;
  current_signal().m_gpio = this;
  m_output_override = outover;
  m_oe_override = oeover;
  m_input_override = inover;
  m_irq_override = irqover;
  update_from_mux();
}

uint32_t GPIO::get_ctrl() const{
  return m_funcsel | (m_output_override << 8) | (m_oe_override << 12) | (m_input_override << 16) | (m_irq_override << 28);
}

uint32_t GPIO::get_status() const{
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

