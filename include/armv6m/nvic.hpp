#pragma once

#include <cstdint>
#include "interrupt.hpp"

namespace ARMv6M
{
  
  class NVIC final: public IInterruptSink{
  public:
    virtual void irq_raised(int irq_num) override final;
    uint32_t get_vt_addr() const { return m_vtor; }
  protected:
  private:

    void set_irq_priority(int irq_num, uint8_t priority);
    uint8_t get_irq_priority(int irq_num) const;

    uint32_t m_vtor;
    InterruptSource *m_irqs[32];
    uint32_t m_irqs_enabled; // bitmask for each irq source
    uint32_t m_irqs_latched; // irq can safely be cleared
    uint32_t m_irqs_pending; // bitmask for each irq source
    uint32_t m_irqs_priority[8]; // priority for each irq source
  };

} // namespace ARMv6M
