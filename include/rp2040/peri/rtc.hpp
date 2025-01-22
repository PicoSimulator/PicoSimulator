#pragma once

#include "rp2040/peripheral.hpp"
#include "armv6m/exception.hpp"
#include "interrupt.hpp"
#include "clock.hpp"

class RTC final : public IPeripheralPort, public IClockable{
public:
  RTC(InterruptSource &irq) : m_irqs{irq} {}
  virtual void tick() override final
  {
    if(m_reg_rtc1 & 0x80000000){
      // m_reg_rtc1 &= ~0x80000000;
      // m_irq.set_pending(1);
    }
  }
protected:
  enum Register{
    CLKDIV_M1 = 0x00,
    SETUP_0 = 0x04,
    SETUP_1 = 0x08,
    CTRL = 0x0c,
    IRQ_SETUP_0 = 0x10,
    IRQ_SETUP_1 = 0x14,
    RTC_1 = 0x18,
    RTC_0 = 0x1c,
    INTR = 0x20,
    INTE = 0x24,
    INTF = 0x28,
    INTS = 0x2c,
  };
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xff){
      case CTRL:
        out = m_reg_ctrl;
        break;
      case RTC_0:
        m_time = m_reg_rtc1;
        m_date = m_reg_rtc0;
        out = m_time;
        break;
      case RTC_1:
        out = m_date;
        break;
      case INTR:
        out = m_irqs.raw();
        break;
      case INTE:
        out = m_irqs.mask();
        break;
      case INTF:
        out = m_irqs.masked();
        break;
      case INTS:
        out = m_irqs.masked();
        break;
      default:
        throw ARMv6M::BusFault{addr};
        return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    switch(addr & 0xff){
      case CLKDIV_M1:
        break;
      case SETUP_0:
        break;
      case SETUP_1:
        break;
      case CTRL:
        m_reg_ctrl = in & ~2;
        if (in&1) m_reg_ctrl |= 2;
        break;
      case RTC_0:
        m_reg_rtc0 = in;
        break;
      case RTC_1:
        m_reg_rtc1 = in;
        break;
      case INTR:
        m_irqs.force(in);
        break;
      case INTE:
        m_irqs.mask(in);
        break;
      case INTF:
        m_irqs.mask(in);
        break;
      case INTS:
        m_irqs.mask(in);
        break;
      default:
        throw ARMv6M::BusFault{addr};
        return PortState::FAULT;
    }
    return PortState::FAULT;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

  static uint16_t date_get_year(uint32_t date) { return (date >> 12) & 0x0fff; }
  static uint8_t date_get_month(uint32_t date) { return (date >> 8) & 0x0f; }
  static uint8_t date_get_day(uint32_t date) { return date & 0x1f; }
  static uint8_t time_get_dotw(uint32_t time) { return (time >> 24) & 0x07; }
  static uint8_t time_get_hour(uint32_t time) { return (time >> 16) & 0x1f; }
  static uint8_t time_get_minute(uint32_t time) { return (time >> 8) & 0x3f; }
  static uint8_t time_get_second(uint32_t time) { return time & 0x3f; }
private:
  uint32_t m_date;
  uint32_t m_time;

  uint32_t m_reg_rtc0;
  uint32_t m_reg_rtc1;
  uint32_t m_reg_ctrl;

  InterruptSourceMulti m_irqs;
};