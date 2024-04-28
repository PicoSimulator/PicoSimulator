#include "rp2040/pad.hpp"

using namespace RP2040;

void Pad::net_state_changed(){
  m_input_value_raw = connected_net()->digital_read();
}

void Pad::pad_state_changed(){
  connected_net()->update();
}

void Pad::update_from_mux(){
  GPIOSignal *sig = m_gpio_funcs[m_funcsel];
  if(sig){
    m_output_value_raw = sig->get_output();
    m_output_enable_raw = sig->get_oe();
  }
}

void GPIOSignal::set_output(bool val){
  m_output_value = val;
  if (m_current_pad)
    m_current_pad->update_from_mux();
}