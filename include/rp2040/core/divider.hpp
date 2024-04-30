#pragma once

#include <cstdint>
#include "clock.hpp"

namespace RP2040::Core
{
  
  class Divider final : public IClockable{
  public:
    virtual void tick() override
    {
      if(m_completion_cycles > 0){
        m_completion_cycles--;
        if(m_completion_cycles == 0){
          m_status |= 1;
        }
      }
    }
    void set_sdividend(uint32_t dividend)
    {
      m_status |= 2;
      m_dividend = dividend;
      start_sdiv();
    }
    void set_sdivisor(uint32_t divisor)
    {
      m_status |= 2;
      m_divisor = divisor;
      start_sdiv();
    }
    void set_udividend(uint32_t dividend)
    {
      m_status |= 2;
      m_dividend = dividend;
      start_udiv();
    }
    void set_udivisor(uint32_t divisor)
    {
      m_status |= 2;
      m_divisor = divisor;
      start_udiv();
    }
    uint32_t get_quotient()
    {
      m_status &= ~2;
      // return 0xaa5555aa;
      return m_quotient;
    }
    uint32_t get_remainder()
    {
      // return 0x55aaaa55;
      return m_remainder;
    }
    uint32_t get_status() 
    {
      return m_status;
    }

  protected:
  private:
    void start_sdiv()
    {
      if (m_divisor == 0) return;
      int64_t dividend = (int32_t)m_dividend;
      int64_t divisor = (int32_t)m_divisor;
      dividend = dividend << 32;
      m_quotient = (dividend / m_divisor) >> 32;
      m_remainder = m_dividend % m_divisor;
      m_completion_cycles = 8;
      m_status &= ~1;
    }
    void start_udiv()
    {
      if (m_divisor == 0) return;
      uint64_t dividend = m_dividend;
      uint64_t divisor = m_divisor;
      dividend = dividend << 32;
      m_quotient = (dividend / divisor) >> 32;
      m_remainder = m_dividend % m_divisor;
      m_completion_cycles = 8;
      m_status &= ~1;
    }
    void abort()
    {
      m_completion_cycles = 0;
    }
    uint8_t m_completion_cycles;
    uint32_t m_dividend, m_divisor;
    uint32_t m_quotient, m_remainder;
    uint32_t m_status;
  };
} // namespace RP2040::Core
