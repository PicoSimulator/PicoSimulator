#pragma once

#include <bitset>
#include <cstdint>
#include <cassert>
#include "common/interrupt.hpp"

namespace ARMv6M
{
  
  class NVIC final : public IResettable{
  public:
    NVIC(InterruptSourceSet &syshandlers, InterruptSourceSet &irqs)
    : m_syshandlers{syshandlers}
    , m_irqs{irqs}
    {
    }
    virtual void reset() override final {
      m_vtor = 0;
      m_irqs_enabled = 0;
      m_shpr2 = 0;
      m_shpr3 = 0;
      for(int i = 0; i < 8; i++){
        m_irqs_priority[i] = 0;
      }
      generate_priority_masks();
      setExceptionActive(ExceptionNumber::NONE, true);
    }
    using exception_bits = std::bitset<48>;
    uint32_t get_vt_addr() const { return m_vtor; }
    enum ExceptionNumber{
      NONE = 0,
      Reset = 1,
      NMI = 2,
      HardFault = 3,
      SVCall = 11,
      PendSV = 14,
      SysTick = 15,
      ExternalInterrupt = 16
    };
    int8_t ExecutionPriority() {
      int8_t highestpri = 4;
      int8_t boostedpri = 4;
      for(int i = 2; i < 48; i++) {
        if (!ExceptionActive(i)) continue;
        highestpri = std::min(highestpri, ExceptionPriority(i));
      }
      if (m_primask) boostedpri = 0;
      int8_t priority = std::min(highestpri, boostedpri);
      return priority;
    }
    bool ExceptionActive(int num) const {
      return m_active_exceptions[num];
    }
    int8_t current_priority() const {
      return m_current_priority;
    }
    void set_current_priority(int8_t prio) {
      m_current_priority = prio;
      m_current_prio_mask = get_priority_mask(current_priority());
    }
    const exception_bits &get_priority_mask(int8_t priority) const {
      return m_priority_masks[priority+3];
    }
    const exception_bits &current_priority_mask() const {
      return m_current_prio_mask;
    }
    void setExceptionActive(ExceptionNumber num, bool active) {
      if(num)
        m_active_exceptions[num] = active;
      if(active) {
        if(num >= ExternalInterrupt)
          m_irqs.clear_pending(1 << (num - ExternalInterrupt));
        else
          m_syshandlers.clear_pending(1 << num);
      }
      
      set_current_priority(ExecutionPriority());
    }
    auto ExceptionActiveBitCount() const {
      return m_active_exceptions.count();
    }

    ExceptionNumber check_pending() const {
      uint64_t bits = uint64_t(uint64_t(m_syshandlers.raised()) | (uint64_t(m_irqs.raised()) << 16));
      exception_bits pending_all = bits;
      pending_all &= current_priority_mask();
      if (!pending_all.any())
        return ExceptionNumber::NONE;
      for(int i = 0; i < 48; i++){
        if (pending_all[i])
          return ExceptionNumber(i);
      }
      assert(false); // should never get here
    }
    uint32_t pending() const {
      return m_irqs.raised();
    }
    uint32_t enabled() const {
      return m_irqs_enabled;
    }
    void set_enable(uint32_t enable) {
      m_irqs_enabled |= enable;
      generate_priority_masks();
    }
    void clear_enable(uint32_t enable) {
      m_irqs_enabled &= ~enable;
      generate_priority_masks();
    }
    void set_pending(uint32_t pending) {
      m_irqs.set_pending(pending);
    }
    void clear_pending(uint32_t pending) {
      m_irqs.clear_pending(pending);
    }
    int8_t ExceptionPriority(int num) const {
      return get_exception_priority(ExceptionNumber(num));
    }
  protected:
  private:
    exception_bits &get_priority_mask(int8_t priority) {
      return m_priority_masks[priority+3];
    }

    void set_irq_priority(int irq_num, int8_t priority);
    int8_t get_irq_priority(int irq_num) const { 
      return m_irqs_priority[irq_num/4] >> ((irq_num % 4) * 8 + 6) & 0x03;
    }
    int8_t get_exception_priority(ExceptionNumber exception_num) const
    {
      switch(exception_num){
        case Reset: return -3;
        case NMI: return -2;
        case HardFault: return -1;
        case SVCall: return (m_shpr2 >> 24) & 0x0f;
        case PendSV: return (m_shpr3 >> 16) & 0x0f;
        case SysTick: return (m_shpr3 >> 24) & 0x0f;
        default: 
          if(exception_num >= ExternalInterrupt)
            return get_irq_priority(exception_num - ExternalInterrupt);
          return 4;
      }
      assert(false);
    }

    void generate_priority_masks()
    {
      for (int i = 1; i < 48; i++) {
        update_priority_masks(ExceptionNumber(i));
      }
      for (int i = -3; i <= 4; i++) {
        std::cout << "Priority mask " << i << ": " << get_priority_mask(i) << std::endl;
      }
    }
    void update_priority_masks(ExceptionNumber num)
    {
      int8_t priority = get_exception_priority(num);
      std::cout << "Updating priority masks for exception " << num << " with priority " << int(priority) << std::endl;
      for(int i = -3; i <= 4; i++) {
        if (priority < i && ((num<ExternalInterrupt)?true:(m_irqs_enabled & (1 << (num-ExternalInterrupt)))))
          get_priority_mask(i)[num] = true;
        else
          get_priority_mask(i)[num] = false;
      }
      set_current_priority(current_priority());
    }

    uint32_t m_vtor;
    InterruptSourceSet &m_syshandlers;
    InterruptSourceSet &m_irqs;
    uint32_t m_irqs_enabled; // bitmask for each irq source
    uint32_t m_shpr2, m_shpr3; // system handler priority registers
    uint32_t m_irqs_priority[8]; // priority for each irq source

    exception_bits m_priority_masks[8];
    exception_bits m_active_exceptions;
    exception_bits m_current_prio_mask;

    bool m_primask;
    int8_t m_current_priority;
  };

} // namespace ARMv6M
