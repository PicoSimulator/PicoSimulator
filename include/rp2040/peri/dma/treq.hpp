#pragma once

#include <cstdint>

#include "clock.hpp"

namespace RP2040::DMA
{
  class TReq final : public IClockable
  {
  public:
    TReq();
    virtual void tick() override;
    operator bool() const { return m_has_ticked_div;}
  protected:
  private:
    uint32_t m_divisor;
    uint32_t m_dividend;
    bool m_has_ticked_div;
  };

} // namespace RP
