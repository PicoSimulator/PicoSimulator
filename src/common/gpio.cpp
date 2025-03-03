#include "common/gpio.hpp"


GPIOSignal &GPIOSignal::dummy(){
  static GPIOSignal dummy;
  return dummy;
}

void GPIOSignal::set_output(bool val){
  m_output_value = val;
  if (m_gpio)
    m_gpio->update_from_internal();
}

void GPIOSignal::set_oe(bool oe){
  m_output_enable = oe;
  if (m_gpio)
    m_gpio->update_from_internal();
}

bool GPIOSignal::get_input() const{
  if (m_input_gpio)
    return m_input_gpio->get_input();
  else if(m_gpio)
    return m_gpio->get_input();
  return false;
}