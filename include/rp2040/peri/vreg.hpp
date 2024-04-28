#pragma once

#include "rp2040/peripheral.hpp"
#include "armv6m/exception.hpp"

class VReg final : public IPeripheralPort{
public:
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    switch (addr & 0x0fff) {
      // case 0x0: // VREG
      //   out = 0;
      //   break;
      // case 0x4: // BOD
      //   out = 0;
      //   break;
      case 0x8: // CHIP_RESET
        out = 0;
        break;
      default:
        throw ARMv6M::BusFault{addr};
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