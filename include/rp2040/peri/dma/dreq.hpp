#pragma once

#include <cstdint>

namespace RP2040::DMA
{
  class DReq
  {
  public:
    DReq() : m_dreq_count(0), m_true_count(0) {}
    void peri_incr(unsigned count=1) { m_dreq_count+=count; m_true_count+=count; }
    void peri_decr() { m_true_count--; }
    void peri_set(uint32_t count) { m_dreq_count = count; m_true_count = count; }
    void dma_sync() { m_dreq_count = m_true_count; }
    void dma_decr() { m_dreq_count--; }
    operator bool() const { return m_dreq_count > 0; }
    uint32_t dreq_count() const { return m_dreq_count; }
    uint32_t true_count() const { return m_true_count; }
  protected:
  private:
    uint32_t m_dreq_count;
    uint32_t m_true_count;
  };

} // namespace RP
