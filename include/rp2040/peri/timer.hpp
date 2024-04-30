#pragma once

#include "rp2040/peripheral.hpp"

class Timer final : public IPeripheralPort{
public:
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
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