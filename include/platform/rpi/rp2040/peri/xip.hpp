#pragma once

#include <array>
#include <cstdint>

#include "memory_device.hpp"
#include "util/saturating_counter.hpp"
// #include "util/array_converter.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "platform/rpi/rp2040/peri/ssi.hpp"
#include <fstream>
#include <span>
#include <cassert>
#include <coroutine>

namespace RP2040{
  class XIP final : public IAsyncReadWritePort<uint32_t>, public IClockable{
  public:
    XIP(SSI &ssi) : m_ssi{ssi}, m_runner{bus_task().get_handle()} {
      for (auto &tag_set : m_cache_tags) {
        for (auto &tag : tag_set) {
          tag = {false, 0};
        }
      }
      m_hit_counter = 0;
      m_acc_counter = 0;
      m_enabled = true;
    }
    virtual void tick() override;
    const std::span<uint8_t> flash() const { return m_ssi.flash(); }
    bool enabled() const { return m_enabled; }

  protected:
  private:
    virtual bool register_op(MemoryOperation &op) override final {
      assert(m_op == nullptr);
      m_op = &op;
      if(m_waiting_on_op) {
        // execute bus task without suspending caller
        // m_waiting_on_op indicates that bus task is currently suspended
        m_runner.resume();
        // check if bus task is completed.
      }
      return m_waiting_on_op;
    }
    bool m_waiting_on_op = false;
    auto next_op(){
      struct awaitable{
        bool await_ready() { 
          return m_xip.m_op != nullptr; 
          // return false;
        }
        MemoryOperation &await_resume() { 
          m_xip.m_waiting_on_op = false;
          return *m_xip.m_op; 
        }
        void await_suspend(std::coroutine_handle<> handle) {
          m_xip.m_waiting_on_op = true;
          m_xip.m_op = nullptr;
        }
        XIP &m_xip;
      };
      if (m_op != nullptr) {
        auto *op = m_op;
        m_op = nullptr;
        op->complete();
      }
      return awaitable{*this};
    }
    
    Task bus_task();
    //hack for now to make this work!
    // std::array<uint8_t, 0x0100'0000> m_flash;


    std::array<uint8_t, 16384> m_data;
    using cache_tag = std::tuple<bool, uint32_t>;
    using tag_set = std::array<cache_tag, 2>;
    std::array<tag_set, 1024> m_cache_tags;
    saturating_counter<uint32_t> m_hit_counter;
    saturating_counter<uint32_t> m_acc_counter;

    static uint32_t cache_tag_decode(uint32_t addr);
    static uint32_t cache_set_decode(uint32_t addr);
    bool cache_set_lookup(uint32_t addr, uint32_t &set, uint32_t &line_no);
    void cache_set_choose_replacement(uint32_t set, uint32_t &line_no);
    void flush();

    uint32_t m_stream_ctr;
    uint32_t m_stream_addr;
    FiFo<uint32_t, 2> m_stream_fifo;

    SSI &m_ssi;
    std::coroutine_handle<> m_runner;
    MemoryOperation *m_op;
    bool m_enabled;
  };

}