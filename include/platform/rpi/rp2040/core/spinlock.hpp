#pragma once

#include <cstdint>

#define ENUM_SPINLOCKS(_) \
  _(0) \
  _(1) \
  _(2) \
  _(3) \
  _(4) \
  _(5) \
  _(6) \
  _(7) \
  _(8) \
  _(9) \
  _(10) \
  _(11) \
  _(12) \
  _(13) \
  _(14) \
  _(15) \
  _(16) \
  _(17) \
  _(18) \
  _(19) \
  _(20) \
  _(21) \
  _(22) \
  _(23) \
  _(24) \
  _(25) \
  _(26) \
  _(27) \
  _(28) \
  _(29) \
  _(30) \
  _(31) \

namespace RP2040::Core
{
  
  class Spinlocks{
  public:
    bool try_lock(uint32_t spinlock_num) { 
      if(m_spinlocks & (1 << spinlock_num)){
        return false;
      }else{
        m_spinlocks |= (1 << spinlock_num);
        return true;
      }
    }
    void unlock(uint32_t spinlock_num) { 
      m_spinlocks &= ~(1 << spinlock_num);
    }
    uint32_t status() const { return m_spinlocks; }
  protected:
  private:
    uint32_t m_spinlocks;
  };
} // namespace RP2040::Core
