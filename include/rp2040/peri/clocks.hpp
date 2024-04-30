#pragma once

#include "rp2040/peripheral.hpp"

class Clocks final : public IPeripheralPort{
public:
protected:
  enum Register{
    CLK_GPOUT0_CTRL = 0x00,
    CLK_GPOUT0_DIV = 0x04,
    CLK_GPOUT0_SELECTED = 0x08,
    CLK_GPOUT1_CTRL = 0x0c,
    CLK_GPOUT1_DIV = 0x10,
    CLK_GPOUT1_SELECTED = 0x14,
    CLK_GPOUT2_CTRL = 0x18,
    CLK_GPOUT2_DIV = 0x1c,
    CLK_GPOUT2_SELECTED = 0x20,
    CLK_GPOUT3_CTRL = 0x24,
    CLK_GPOUT3_DIV = 0x28,
    CLK_GPOUT3_SELECTED = 0x2c,
    CLK_REF_CTRL = 0x30,
    CLK_REF_DIV = 0x34,
    CLK_REF_SELECTED = 0x38,
    CLK_SYS_CTRL = 0x3c,
    CLK_SYS_DIV = 0x40,
    CLK_SYS_SELECTED = 0x44,
    CLK_PERI_CTRL = 0x48,
    CLK_PERI_DIV = 0x4c,
    CLK_PERI_SELECTED = 0x50,
    CLK_USB_CTRL = 0x54,
    CLK_USB_DIV = 0x58,
    CLK_USB_SELECTED = 0x5c,
    CLK_ADC_CTRL = 0x60,
    CLK_ADC_DIV = 0x64,
    CLK_ADC_SELECTED = 0x68,
    CLK_RTC_CTRL = 0x6c,
    CLK_RTC_DIV = 0x70,
    CLK_RTC_SELECTED = 0x74,
    CLK_SYS_RESUS_CTRL = 0x78,
    CLK_SYS_RESUS_STATUS = 0x7c,
    FC0_REF_KHZ = 0x80,
    FC0_MIN_KHZ = 0x84,
    FC0_MAX_KHZ = 0x88,
    FC0_DELAY = 0x8c,
    FC0_INTERVAL = 0x90,
    FC0_SRC = 0x94,
    FC0_STATUS = 0x98,
    FC0_RESULT = 0x9c,
    WAKE_EN0 = 0xa0,
    WAKE_EN1 = 0xa4,
    SLEEP_EN0 = 0xa8,
    SLEEP_EN1 = 0xac,
    ENABLED0 = 0xb0,
    ENABLED1 = 0xb4,
    INTR = 0xb8,
    INTE = 0xbc,
    INTF = 0xc0,
    INTS = 0xc4,
  };
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xfc) {
      case 0x38: out = 1; break; // CLK_REF_SELECTED
      case 0x44: out = 1; break; // CLK_SYS_SELECTED
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