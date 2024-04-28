#pragma once

#include <cstdint>
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

// template<class T>
// class BusAwaitable
// {
// public:
// protected:
// private:
//   struct promise_type;
//   using Handle = std::coroutine<promise_type>;
//   struct promise_type{

//   };
// };

template<class Addr>
class IAsyncReadPort{
public:
  virtual Awaitable<uint8_t> read_byte(Addr addr) = 0;
  virtual Awaitable<uint16_t> read_halfword(Addr addr) = 0;
  virtual Awaitable<uint32_t> read_word(Addr addr) = 0;
protected:
private:
};

template<class Addr>
class IAsyncWritePort{
public:
  virtual Awaitable<void> write_byte(Addr addr, uint8_t in) = 0;
  virtual Awaitable<void> write_halfword(Addr addr, uint16_t in) = 0;
  virtual Awaitable<void> write_word(Addr addr, uint32_t in) = 0;
protected:
private:
};

template<class Addr>
class IAsyncReadWritePort : public IAsyncReadPort<Addr>, public IAsyncWritePort<Addr>{};

// template<class Addr>
// class AsyncWriteAdapter : public IWritePort<Addr>, public IAsyncReadWritePort<Addr>{
// public:
//   virtual Awaitable<void> write_byte(Addr addr, uint8_t in) override final
//   {
//     co_return (addr, in);
//   }
// };

using IMemoryDevice = IReadWritePort<uint32_t>;