#pragma once

#include "rp2040/peripheral.hpp"

class XOSC final : public IPeripheralPort{
public:
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xff) {
      case 0x00: out = 0x00d1'eaa0; break; // xosc disabled?
      case 0x04: out = 0x8000'0000; break; // fake oscillator stable
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::SUCCESS;
  }
  virtual PortState xor_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::FAULT;
  }
  virtual PortState set_bits_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::FAULT;
  }
  virtual PortState clear_bits_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::FAULT;
  }

private:

};