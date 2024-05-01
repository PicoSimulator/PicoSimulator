#pragma once

#include "rp2040/peripheral.hpp"
#include "interrupt.hpp"

#define ENUM_ALARMS(o) \
  o(0) \
  o(1) \
  o(2) \
  o(3)

class Timer final : public IPeripheralPort, public IClockable{
public:
  Timer() : m_irq_alarm0{"TIMER_IRQ_0"},
            m_irq_alarm1{"TIMER_IRQ_1"},
            m_irq_alarm2{"TIMER_IRQ_2"},
            m_irq_alarm3{"TIMER_IRQ_3"}
  {
    m_time = 0;
    m_time_load = 0;
    m_alarm0 = 0;
    m_alarm1 = 0;
    m_alarm2 = 0;
    m_alarm3 = 0;
    m_armed_mask = 0;
    m_paused = false;
  }
  virtual void tick() override final
  {
    if (m_paused) {
      return;
    }
    m_time++;

    #define CHECK_ALARM(n) \
      if (m_armed_mask & (1 << n) && (m_time & 0xffff'ffff) == m_alarm##n) { \
        m_armed_mask &= ~(1 << n); \
        m_irq_alarm##n.raise(); \
      }
    ENUM_ALARMS(CHECK_ALARM)
    #undef CHECK_ALARM
    
  }
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xff) {
      case TIMEHW:
        out = 0;
        break;
      case TIMELW:
        out = 0;
        break;
      case TIMEHR:
        out = m_time_load >> 32;
        break;
      case TIMELR:
        m_time_load = m_time;
        out = m_time_load & 0xffff'ffff;
        break;
      case ALARM0:
        out = m_alarm0;
        break;
      case ALARM1:
        out = m_alarm1;
        break;
      case ALARM2:
        out = m_alarm2;
        break;
      case ALARM3:
        out = m_alarm3;
        break;
      case ARMED:
        out = m_armed_mask;
        break;
      case TIMERAWH:
        out = m_time >> 32;
        break;
      case TIMERAWL:
        out = m_time & 0xffff'ffff;
        break;
      case DBGPAUSE:
        out = 0;
        break;
      case PAUSE:
        out = m_paused;
        break;
      case INTR:
        out = 0;
        break;
      case INTE:
        out = 0;
        break;
      case INTF:
        out = 0;
        break;
      case INTS:
        out = 0;
        break;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    switch(addr & 0xff) {
      case TIMEHW:
        m_time_load |= uint64_t{in} << 32;
        m_time = m_time_load;
        break;
      case TIMELW:
        m_time_load = in;
        break;
      case ALARM0:
        m_alarm0 = in;
        m_armed_mask |= 1 << 0;
        break;
      case ALARM1:
        m_alarm1 = in;
        m_armed_mask |= 1 << 1;
        break;
      case ALARM2:
        m_alarm2 = in;
        m_armed_mask |= 1 << 2;
        break;
      case ALARM3:
        m_alarm3 = in;
        m_armed_mask |= 1 << 3;
        break;
      case ARMED:
        m_armed_mask &= ~in;
        break;
        
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

private:
  enum Register{
    TIMEHW = 0x00,
    TIMELW = 0x04,
    TIMEHR = 0x08,
    TIMELR = 0x0C,
    ALARM0 = 0x10,
    ALARM1 = 0x14,
    ALARM2 = 0x18,
    ALARM3 = 0x1C,
    ARMED = 0x20,
    TIMERAWH = 0x24,
    TIMERAWL = 0x28,
    DBGPAUSE = 0x2C,
    PAUSE = 0x30,
    INTR = 0x34,
    INTE = 0x38,
    INTF = 0x3C,
    INTS = 0x40,
  };

  uint64_t m_time;
  uint64_t m_time_load;

  uint32_t m_alarm0;
  uint32_t m_alarm1;
  uint32_t m_alarm2;
  uint32_t m_alarm3;
  uint32_t m_armed_mask;
  bool m_paused;
  
  
  InterruptSource m_irq_alarm0;
  InterruptSource m_irq_alarm1;
  InterruptSource m_irq_alarm2;
  InterruptSource m_irq_alarm3;
  
  


};