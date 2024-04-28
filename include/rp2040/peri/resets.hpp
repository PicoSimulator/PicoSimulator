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