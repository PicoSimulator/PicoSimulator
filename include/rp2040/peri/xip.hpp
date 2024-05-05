#pragma once

#include <array>
#include <cstdint>

#include "memory_device.hpp"
#include "util/saturating_counter.hpp"
// #include "util/array_converter.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "rp2040/peri/ssi.hpp"
#include <fstream>
#include <span>
#include <cassert>
#include <coroutine>

namespace RP2040{
  class XIP final : public IAsyncReadWritePort<uint32_t>, public IClockable{
  public:
    XIP(SSI &ssi) : m_ssi{ssi}, m_runner{run().get_handle()} {
      for (auto &tag_set : m_cache_tags) {
        for (auto &tag : tag_set) {
          tag = {false, 0};
        }
      }
    }
    void load_binary_data(const std::span<uint8_t> data){
      std::copy(data.begin(), data.end(), m_flash.begin());
    }
    virtual void tick() override;

  protected:
  private:
    virtual void register_op(MemoryOperation &op) override final {
      assert(m_op == nullptr);
      m_op = &op;
      m_runner.resume();
    }
    virtual void deregister_op(MemoryOperation &op) override final {
      assert(m_op == &op);
      m_op = nullptr;
    }
    auto next_op(){
      struct awaitable{
        bool await_ready() { return m_xip.m_op != nullptr; }
        MemoryOperation &await_resume() { return *m_xip.m_op; }
        void await_suspend(std::coroutine_handle<> handle) {
        }
        XIP &m_xip;
      };
      return awaitable{*this};
    }
    
    Task run(){
      while(true) {
        auto &op = co_await next_op();
        // std::cout << "got op" << std::endl;
        switch (op.optype) {
          case MemoryOperation::READ_BYTE:
            co_await op.return_value(m_flash[op.addr&0x00ff'ffff]);
            break;
          case MemoryOperation::READ_HALFWORD:
            co_await op.return_value(*(uint16_t*)&m_flash[op.addr&0x00ff'ffff]);
            break;
          case MemoryOperation::READ_WORD:
            if ((op.addr & 0x0f00'0000) ==  0x0800'0000) {
              uint32_t out;
              m_ssi.read_word(op.addr, out);
              co_await op.return_value(out);
            } else {
              co_await op.return_value(*(uint32_t*)&m_flash[op.addr&0x00ff'ffff]);
            }
            break;
          case MemoryOperation::WRITE_BYTE:
            // m_flash[op.addr] = op.data;
            co_await op.return_void();
            break;
          case MemoryOperation::WRITE_HALFWORD:
            // *(uint16_t*)&m_flash[op.addr] = op.data;
            co_await op.return_void();
            break;
          case MemoryOperation::WRITE_WORD:
            // *(uint32_t*)&m_flash[op.addr] = op.data;
            if ((op.addr & 0x0f00'0000) ==  0x0800'0000) {
              m_ssi.write_word(op.addr, op.data);
            }
            co_await op.return_void();
            break;
        }
      }
    }
    //hack for now to make this work!
    std::array<uint8_t, 0x0100'0000> m_flash;


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
  };

}