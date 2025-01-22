#pragma once

namespace RP2040{
  class RP2040;
}

#include "rp2040/peripheral.hpp"

#define ENUM_CLOCKS(o) \
o(CLK_REF, clk_ref) \
o(CLK_SYS, clk_sys) \
o(CLK_PERI, clk_peri) \
o(CLK_USB, clk_usb) \
o(CLK_ADC, clk_adc) \
o(CLK_RTC, clk_rtc) \
o(CLK_GPOUT0, clk_gpout0) \
o(CLK_GPOUT1, clk_gpout1) \
o(CLK_GPOUT2, clk_gpout2) \
o(CLK_GPOUT3, clk_gpout3)



class Clocks final : public IPeripheralPort{
public:
  Clocks(RP2040::RP2040 &rp2040) : m_rp2040{rp2040} {}
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    std::cout << "Clocks::read_word_internal(0x" << std::hex << addr << ")" << std::dec << std::endl;
    out = 0xffff'ffff; // hack
    switch(addr & 0xfc) {
      case CLK_REF_CTRL: out = m_clk_ref_ctrl; break;
      case CLK_REF_SELECTED: out = m_clk_ref_selected; break; // CLK_REF_SELECTED
      case CLK_SYS_CTRL: out = m_clk_sys_ctrl; break;
      case CLK_SYS_SELECTED: out = m_clk_sys_selected; break; // CLK_SYS_SELECTED
      case CLK_PERI_CTRL: ; break;
      case CLK_PERI_SELECTED: out = 1; break; // CLK_PERI_SELECTED has no glitchless mux
      case CLK_RTC_CTRL: break;
      case CLK_RTC_SELECTED: out = 1; break;
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    std::cout << "Clocks::read_word_internal_pure(0x" << std::hex << addr << ")" << std::dec << std::endl;
    uint32_t out = 0xffff'ffff; // hack
    switch(addr & 0xfc) {
      case CLK_REF_CTRL: out = m_clk_ref_ctrl; break;
      case CLK_REF_SELECTED: out = m_clk_ref_selected; break; // CLK_REF_SELECTED
      case CLK_SYS_CTRL: out = m_clk_sys_ctrl; break;
      case CLK_SYS_SELECTED: out = m_clk_sys_selected; break; // CLK_SYS_SELECTED
      case CLK_PERI_CTRL: ; break;
      case CLK_PERI_SELECTED: out = 1; break; // CLK_PERI_SELECTED has no glitchless mux
    }
    return out;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    std::cout << "Clocks::write_word_internal(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
    switch(addr & 0xfc) {
      case CLK_REF_CTRL: m_clk_ref_ctrl = in; m_clk_ref_selected = (1 << (in & 0x03)); break;
      case CLK_SYS_CTRL: m_clk_sys_ctrl = in; m_clk_sys_selected = (1 << (in & 0x01)); break;
      case CLK_PERI_CTRL: ; break;
      case CLK_RTC_CTRL: ; break;
    }
    return PortState::SUCCESS;
  }

private:
  ClockDiv &clk_gpout0();
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

  uint32_t m_clk_ref_ctrl = 0;
  uint32_t m_clk_ref_selected = 1;
  uint32_t m_clk_sys_ctrl = 0;
  uint32_t m_clk_sys_selected = 1;
  uint32_t m_clk_peri_selected = 1;


  RP2040::RP2040 &m_rp2040;


};

#include "rp2040.hpp"