#pragma once

#include <cstdint>
#include <cassert>
#include <coroutine>
#include "async.hpp"

enum class PortState{
  SUCCESS,
  STALL,
  FAULT,
};

template<class Addr>
class IReadPort{
public:
  virtual PortState read_byte(Addr addr, uint8_t &out) = 0;
  virtual PortState read_halfword(Addr addr, uint16_t &out) = 0;
  virtual PortState read_word(Addr addr, uint32_t &out) = 0;
protected:
private:
};
template<class Addr>
class IWritePort{
public:
  virtual PortState write_byte(Addr addr, uint8_t in) = 0;
  virtual PortState write_halfword(Addr addr, uint16_t in) = 0;
  virtual PortState write_word(Addr addr, uint32_t in) = 0;
protected:
private:
};

template<class Addr>
class IReadWritePort : public IReadPort<Addr>, public IWritePort<Addr>{};


template<class Addr, class Device>
using AddressDecoder = std::tuple<Device, Addr>(*)(Addr addr);

template<class Addr, class Device, AddressDecoder<Addr, Device> decoder, class ...Devices>
class IBusSplitter{};

// template<class Addr, class Data = uint32_t>
// struct MemoryOp{
//   enum OpType{
//     READ_BYTE,
//     READ_HALFWORD,
//     READ_WORD,
//     WRITE_BYTE,
//     WRITE_HALFWORD,
//     WRITE_WORD,
//   };
//   Addr addr;
//   Data data;
//   bool read_not_write;
//   uint32_t size;
//   std::coroutine_handle<> m_caller;
//   IAsyncReadPort<Addr> &m_port;
//   bool await_ready() { return false; }
//   void await_suspend(std::coroutine_handle<> handle) {}
//   Data await_resume() { return data; }
// };

template<class Addr>
class IAsyncReadWritePort;

struct MemoryOperation{
  struct [[nodiscard]] awaiter {
    std::coroutine_handle<> caller;
    bool await_ready() { 
      return false; 
    }
    void await_suspend(std::coroutine_handle<> handle) {
      // std::cout << "resuming handle " << uintptr_t(this) << std::endl;
      // allow the caller to continue until next suspend point.
      handle.resume();
      // std::cout << "resuming caller " << uintptr_t(this) << std::endl;
      caller.resume();
    }
    void await_resume() {}
  };
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
  IAsyncReadWritePort<uint32_t> &m_port;
  std::coroutine_handle<> m_caller = nullptr;
  void return_void() ;
  void return_value(uint32_t value);
  void complete() {
    if (m_caller)
      m_caller.resume();
  }
  bool await_ready();
  void await_suspend(std::coroutine_handle<> handle);
  uint32_t await_resume() {
    return data;
  }


};

template<class Addr>
class IAsyncReadWritePort{
public:
  MemoryOperation read_byte(Addr addr) {
    // std::cout << "read_byte" << std::endl;
    return MemoryOperation{addr, 0, MemoryOperation::READ_BYTE, *this};
  }
  MemoryOperation read_halfword(Addr addr) {
    // std::cout << "read_halfword" << std::endl;
    return MemoryOperation{addr, 0, MemoryOperation::READ_HALFWORD, *this};
  }
  MemoryOperation read_word(Addr addr) {
    // std::cout << "read_word " << std::hex << addr << std::endl;
    return MemoryOperation{addr, 0, MemoryOperation::READ_WORD, *this};
  }
  MemoryOperation write_byte(Addr addr, uint8_t in) {
    // std::cout << "write_byte" << std::endl;
    return MemoryOperation{addr, in, MemoryOperation::WRITE_BYTE, *this};
  }
  MemoryOperation write_halfword(Addr addr, uint16_t in) {
    // std::cout << "write_halfword" << std::endl;
    return MemoryOperation{addr, in, MemoryOperation::WRITE_HALFWORD, *this};
  }
  MemoryOperation write_word(Addr addr, uint32_t in) {
    // std::cout << "write_word " << std::hex << addr << ":" << in <<  std::endl;
    return MemoryOperation{addr, in, MemoryOperation::WRITE_WORD, *this};
  }
  virtual bool register_op(MemoryOperation &op) = 0;
  // virtual void deregister_op(MemoryOperation &op) = 0;
protected:
private:

};

class ISingleAsyncReadWritePort : public IAsyncReadWritePort<uint32_t>{
public:
protected:
private:
};
template<int N>
class IMultiAsyncReadWritePort : public IAsyncReadWritePort<uint32_t>{
public:
protected:
private:
};

inline void MemoryOperation::return_void() {
  assert(is_write());
  // DO NOT ENABLE THESE PRINTS THEY ADD 200MB TO THE LOGS PER MCYCLES
  // std::cout << "MemoryOperation::return_void" << std::endl;
  // m_port.deregister_op(*this);
  // m_caller.resume();
  // return awaiter{m_caller};
}

inline void MemoryOperation::return_value(uint32_t value) {
  assert(is_read());
  // DO NOT ENABLE THESE PRINTS THEY ADD 200MB TO THE LOGS PER MCYCLES
  // std::cout << "MemoryOperation::return_value " << value << std::endl;
  // m_port.deregister_op(*this);
  data = value;
  // m_caller.resume();
  // return awaiter{m_caller};
}

inline void MemoryOperation::await_suspend(std::coroutine_handle<> h) {
  m_caller = h;
  // m_port.register_op(*this);
}

inline bool MemoryOperation::await_ready() {
  // attempt to complete the operation immediately
  // by registering the operation and observing the
  // return value
  return m_port.register_op(*this);
}

// template<class Addr>
// class AsyncWriteAdapter : public IWritePort<Addr>, public IAsyncReadWritePort<Addr>{
// public:
//   virtual Awaitable<void> write_byte(Addr addr, uint8_t in) override final
//   {
//     co_return (addr, in);
//   }
// };

using IMemoryDevice = IReadWritePort<uint32_t>;