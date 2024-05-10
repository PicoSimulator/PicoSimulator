#pragma once

#include <cstdint>
#include <functional>
#include "util/saturating_counter.hpp"
#include <iostream>
#include <iomanip>
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

  class DReqSource {
  public:
    uint32_t count() const { return m_dreq_assert_count; }
    uint32_t operator++(int) { m_dreq_assert_count++; return m_dreq_assert_count; }
    uint32_t operator++() { m_dreq_assert_count++; return m_dreq_assert_count-1; }
    uint32_t operator+=(uint32_t v) { m_dreq_assert_count += v; return m_dreq_assert_count; }
    virtual void sync() = 0;
  protected:
  private:
    // this only ever INCREASES
    // it keeps track of how many cycles the DREQ 
    // signal has ever been asserted for
    uint32_t m_dreq_assert_count;
  };

  class NullDReqSource final : public DReqSource {
  public:
    void sync() override final {}
    static NullDReqSource &get() { return s_nulldreq;}
  private:
    static NullDReqSource s_nulldreq;
  };

  class DReqSink final {
  public:
    DReqSink(DReqSource &source) : m_source{source}{
      std::cout << "DReqSink sourc: " << std::hex << &source << std::endl;
    }
    void sync() { 
      std::cout << "a" << std::hex << uintptr_t(&m_source.get()) << std::endl;
      m_tracking_count = m_source.get().count();
      m_count = 0; 
      m_source.get().sync();
      track();
    }
    DReqSource& source() { return m_source; }
    void connect(DReqSource &source) { m_source = source; }
    uint32_t operator--(int) { m_count--; track(); return *this; }
    uint32_t operator--() { uint32_t v = *this; m_count--; track(); return v;}
    operator uint32_t() { track(); return m_count; }
    operator bool() const { return uint32_t{*this} > 0; }
  protected:
  private:
    void track() {
      uint32_t source_count = m_source.get().count();
      uint32_t diff = source_count - m_tracking_count;
      m_tracking_count = source_count;
      m_count += diff;
    }
    std::reference_wrapper<DReqSource> m_source;
    // this only ever INCREASES to match m_source->m_count
    uint32_t m_tracking_count;
    saturating_counter<uint8_t> m_count;
  };

} // namespace RP
