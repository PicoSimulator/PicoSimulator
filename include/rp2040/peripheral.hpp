#pragma once

#include "memory_device.hpp"
#include "armv6m/exception.hpp"

#include <coroutine>
#include <cassert>

class IPeripheralPort : public IReadWritePort<uint32_t>{
public:
  virtual PortState read_byte(uint32_t addr, uint8_t &out) override final
  {
    return PortState::FAULT;
  }
  virtual PortState read_halfword(uint32_t addr, uint16_t &out) override final
  {
    return PortState::FAULT;
  }
  virtual PortState read_word(uint32_t addr, uint32_t &out) override final
  {
    return read_word_internal(addr, out);
  }
  virtual PortState write_byte(uint32_t addr, uint8_t in) override final
  {
    return PortState::FAULT;
  }
  virtual PortState write_halfword(uint32_t addr, uint16_t in) override final
  {
    return PortState::FAULT;
  }
  virtual PortState write_word(uint32_t addr, uint32_t in) override final
  {
    // std::cout << "IPeripheralPort::write_word(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
    switch(addr&0x0000'3000) {
      case 0x0000'0000:
        return write_word_internal(addr, in);
      case 0x0000'1000:
        return write_word_internal(addr, in ^ read_word_internal_pure(addr));
      case 0x0000'2000:
        return write_word_internal(addr, in | read_word_internal_pure(addr));
      case 0x0000'3000:
        return write_word_internal(addr, ~in & read_word_internal_pure(addr));
    }
    return PortState::FAULT;
  }
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) = 0;
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) = 0;
  virtual uint32_t read_word_internal_pure(uint32_t addr) const = 0;
private:

};


class IInterposedPeripheralPort : public IAsyncReadWritePort<uint32_t>{
public:
  Awaitable<uint8_t> read_byte_internal2(uint32_t addr)
  {
    throw ARMv6M::BusFault{addr};
  }
  Awaitable<uint16_t> read_halfword_internal2(uint32_t addr)
  {
    throw ARMv6M::BusFault{addr};
  }
  Awaitable<uint32_t> read_word_internal2(uint32_t addr)
  {
    uint32_t out;
    if (read_word_internal(addr, out) != PortState::SUCCESS) {
      throw ARMv6M::BusFault{addr};
    }
    co_return out;
  }
  Awaitable<void> write_byte_internal2(uint32_t addr, uint8_t in)
  {
    throw ARMv6M::BusFault{addr};
  }
  Awaitable<void> write_halfword_internal2(uint32_t addr, uint16_t in)
  {
    throw ARMv6M::BusFault{addr};
  }
  Awaitable<void> write_word_internal2(uint32_t addr, uint32_t in)
  {
    switch(addr&0x0000'3000) {
      case 0x0000'0000:
        write_word_internal(addr, in);
        break;
      // case 0x0000'1000:
      //   return xor_word_internal(addr, in);
      // case 0x0000'2000:
      //   return set_bits_word_internal(addr, in);
      // case 0x0000'3000:
      //   return clear_bits_word_internal(addr, in);
      default:
        throw ARMv6M::BusFault{addr};
    }
    co_return;
  }
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) = 0;
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) = 0;
  virtual bool register_op(MemoryOperation &op) override final{
    assert(m_op == nullptr);
    m_op = &op;
    if(m_waiting_on_op)
      m_runner.resume();
    return m_waiting_on_op;
  }
  // virtual void deregister_op(MemoryOperation &op) override final{
  //   assert(m_op == &op);
  //   m_op = nullptr;
  //   m_waiting_on_op = false;
  // }
private:
  MemoryOperation *m_op;
  std::coroutine_handle<> m_runner;
  bool m_waiting_on_op;
  auto next_op(){
    struct awaitable{
      bool await_ready() { 
        return m_bus.m_op != nullptr; 
      }
      MemoryOperation &await_resume() { 
        m_bus.m_waiting_on_op = false;
        return *m_bus.m_op; 
      }
      void await_suspend(std::coroutine_handle<> handle) {
        m_bus.m_waiting_on_op = true;
        m_bus.m_op = nullptr;
      }
      IInterposedPeripheralPort &m_bus;
    };
    if (m_op != nullptr) {
      auto *op = m_op;
      m_op = nullptr;
      op->complete();
    }
    return awaitable{*this};
  }
  Task bus_task()
  {
    while(true) {
      auto &op = co_await next_op();
      switch(op.optype) {
        case MemoryOperation::READ_BYTE:
          op.return_value(co_await read_byte_internal2(op.addr));
          break;
        case MemoryOperation::READ_HALFWORD:
          op.return_value(co_await read_halfword_internal2(op.addr));
          break;
        case MemoryOperation::READ_WORD:
          op.return_value(co_await read_word_internal2(op.addr));
          break;
        case MemoryOperation::WRITE_BYTE:
          co_await write_byte_internal2(op.addr, op.data);
          op.return_void();
          break;
        case MemoryOperation::WRITE_HALFWORD:
          co_await write_halfword_internal2(op.addr, op.data);
          op.return_void();
          break;
        case MemoryOperation::WRITE_WORD:
          co_await write_word_internal2(op.addr, op.data);
          op.return_void();
          break;  
      }
    }
  }

};

