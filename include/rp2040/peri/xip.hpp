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
  class XIP final : /* public IAsyncReadWritePort<uint32_t>, */ public IClockable{
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
    // virtual Awaitable<uint8_t> read_byte(uint32_t addr) override;
    // virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override;
    // virtual Awaitable<uint32_t> read_word(uint32_t addr) override;
    // virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override;
    // virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override;
    // virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override;
    struct MemoryOperation{
      uint32_t addr;
      uint32_t data;
      enum OpType{
        READ_BYTE,
        READ_HALFWORD,
        READ_WORD,
        WRITE_BYTE,
        WRITE_HALFWORD,
        WRITE_WORD,
      } optype;
      bool is_read() const {
        return optype == READ_BYTE || optype == READ_HALFWORD || optype == READ_WORD;
      }
      bool is_write() const {
        return optype == WRITE_BYTE || optype == WRITE_HALFWORD || optype == WRITE_WORD;
      }
      std::coroutine_handle<> m_caller;
      XIP &m_xip;
      void return_void() {
        assert(is_write());
        std::cout << "return_void" << std::endl;
        m_xip.deregister_op(*this);
        m_caller.resume();
      }
      template<typename T>
      void return_value(T&& value)
      {
        assert(is_read());
        std::cout << "return_value" << std::endl;
        m_xip.deregister_op(*this);
        data = value;
        m_caller.resume();
      }
      bool await_ready() { 
        return false;
      }
      void await_suspend(std::coroutine_handle<> handle) {
        m_caller = handle;
        m_xip.register_op(*this);
      }
      uint32_t await_resume() {
        return data;
      }


    };
    MemoryOperation read_byte(uint32_t addr)
    {
      std::cout << "read_byte" << std::endl;
      return MemoryOperation{addr, 0, MemoryOperation::READ_BYTE, nullptr, *this};
    }
    auto read_halfword(uint32_t addr)
    {
      std::cout << "read_halfword" << std::endl;
      return MemoryOperation{addr, 0, MemoryOperation::READ_HALFWORD, nullptr, *this};
    }
    auto read_word(uint32_t addr)
    {
      std::cout << "read_word" << std::endl;
      return MemoryOperation{addr, 0, MemoryOperation::READ_WORD, nullptr, *this};
    }
    auto write_byte(uint32_t addr, uint8_t in)
    {
      std::cout << "write_byte" << std::endl;
      return MemoryOperation{addr, in, MemoryOperation::WRITE_BYTE, nullptr, *this};
    }
    auto write_halfword(uint32_t addr, uint16_t in)
    {
      std::cout << "write_halfword" << std::endl;
      return MemoryOperation{addr, in, MemoryOperation::WRITE_HALFWORD, nullptr, *this};
    }
    auto write_word(uint32_t addr, uint32_t in)
    {
      std::cout << "write_word " << std::hex << addr <<  std::endl;
      return MemoryOperation{addr, in, MemoryOperation::WRITE_WORD, nullptr, *this};
    }

  protected:
  private:
    void register_op(MemoryOperation &op){
      assert(m_op == nullptr);
      m_op = &op;
      m_runner.resume();
    }
    void deregister_op(MemoryOperation &op){
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
        std::cout << "got op" << std::endl;
        switch (op.optype) {
          case MemoryOperation::READ_BYTE:
            op.return_value(m_flash[op.addr&0x00ff'ffff]);
            break;
          case MemoryOperation::READ_HALFWORD:
            op.return_value(*(uint16_t*)&m_flash[op.addr&0x00ff'ffff]);
            break;
          case MemoryOperation::READ_WORD:
            if ((op.addr & 0x0f00'0000) ==  0x0800'0000) {
              uint32_t out;
              m_ssi.read_word(op.addr, out);
              op.return_value(out);
            } else {
              op.return_value(*(uint32_t*)&m_flash[op.addr&0x00ff'ffff]);
            }
            break;
          case MemoryOperation::WRITE_BYTE:
            // m_flash[op.addr] = op.data;
            op.return_void();
            break;
          case MemoryOperation::WRITE_HALFWORD:
            // *(uint16_t*)&m_flash[op.addr] = op.data;
            op.return_void();
            break;
          case MemoryOperation::WRITE_WORD:
            // *(uint32_t*)&m_flash[op.addr] = op.data;
            if ((op.addr & 0x0f00'0000) ==  0x0800'0000) {
              m_ssi.write_word(op.addr, op.data);
            }
            op.return_void();
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