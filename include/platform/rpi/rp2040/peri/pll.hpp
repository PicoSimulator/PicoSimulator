#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"

class PLL final : public IPeripheralPort{
public:
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xfc) {
      case 0x00: out = (1<<31); break; // CS
      case 0x04: out = 1; break; // PWR
      case 0x08: out = 1; break; // FBDIV_INT
      case 0x0c: out = 1; break; // PRIM
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

private:

};