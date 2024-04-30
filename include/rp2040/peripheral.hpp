#pragma once

#include "memory_device.hpp"
#include "armv6m/exception.hpp"

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
    std::cout << "IPeripheralPort::write_word(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
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
  virtual Awaitable<uint8_t> read_byte(uint32_t addr) override final
  {
    throw ARMv6M::BusFault{addr};
  }
  virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override final
  {
    throw ARMv6M::BusFault{addr};
  }
  virtual Awaitable<uint32_t> read_word(uint32_t addr) override final
  {
    uint32_t out;
    if (read_word_internal(addr, out) != PortState::SUCCESS) {
      throw ARMv6M::BusFault{addr};
    }
    co_return out;
  }
  virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override final
  {
    throw ARMv6M::BusFault{addr};
  }
  virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override final
  {
    throw ARMv6M::BusFault{addr};
  }
  virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override final
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
  }
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) = 0;
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) = 0;
private:

};

