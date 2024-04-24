#pragma once

#include <cstdint>

namespace ARMv6M
{
  
  class NVIC {
  public:
    uint32_t get_vt_addr() const { return m_vtor; }
  protected:
  private:
    uint32_t m_vtor;
  };

} // namespace ARMv6M
