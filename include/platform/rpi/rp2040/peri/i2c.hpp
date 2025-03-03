#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "common/interrupt.hpp"
#include "platform/rpi/rp2040/pad.hpp"
#include "platform/rpi/rp2040/gpio.hpp"

namespace RP2040 {
  class RP2040;
}

class I2C final/*?*/ : public IPeripheralPort {
public:
  I2C(InterruptSource &irq_source) 
  : m_irqs(irq_source) 
  {}
  GPIOSignal &SDA() { return m_sda; }
  GPIOSignal &SCL() { return m_scl; }


protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    std::cerr << "I2C::read_word_internal(0x" << std::hex << addr << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    std::cerr << "I2C::write_word_internal(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {return 0;}

private:
  InterruptSourceMulti m_irqs;
  GPIOSignal m_sda;
  GPIOSignal m_scl;
};