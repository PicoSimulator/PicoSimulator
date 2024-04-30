#pragma once

#include "rp2040/peripheral.hpp"

class Resets final : public IPeripheralPort{
public:
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    switch (addr & 0x0fff) {
      case 0x0: // RESET
        out = 0;
        break;
      case 0x4: // WDSEL
        out = 0;
        break;
      case 0x8: // RESET_DONE
        out = 0xffff'ffff;
        break;
      default:
        return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    return PortState::FAULT;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

private:

};