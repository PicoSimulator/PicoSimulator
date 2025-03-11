#pragma once

namespace RP2040{
  class RP2040;
}
  
#include "platform/rpi/rp2040/peripheral.hpp"

namespace RP2040::Peripheral {
  class ROsc final : public IPeripheralPort {
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
    {
      out = 0;
      switch(addr & 0xff){
        case 0x00: // CTRL
        case 0x04: // FREQA
        case 0x08: // FREQB
        case 0x0c: // DORMANT
        case 0x10: // DIV
        case 0x14: // PHASE
        case 0x18: // STATUS
        case 0x1c: // RANDOMBIT
        case 0x20: // COUNT
        default:
          return PortState::FAULT;
      }
      return PortState::SUCCESS;
    }
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
    {
      switch(addr & 0xff){
        case 0x00: // CTRL
        case 0x04: // FREQA
        case 0x08: // FREQB
        case 0x0c: // DORMANT
        case 0x10: // DIV
        case 0x14: // PHASE
        case 0x18: // STATUS
        case 0x1c: // RANDOMBIT
        case 0x20: // COUNT
        default:
          return PortState::FAULT;
      }
      return PortState::FAULT;
    }
    virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
    {
      return 0;
    }
  private:
    bool m_badwrite;

  };
}