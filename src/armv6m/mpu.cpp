#include "armv6m/core.hpp"

using namespace ARMv6M;

// currently just a transparent interface to the next bus layer

ARMv6MCore::MPU::MPU(IAsyncReadWritePort<uint32_t> &bus, ARMv6MCore &core) 
: m_bus_interface{bus}
, m_core{core}
, m_runner{bus_task().get_handle()}
{
}

Task ARMv6MCore::MPU::bus_task()
{
  uint32_t out;
  while(true) {
    auto &op = co_await next_op();
    // std::cout << "MPU::bus_task() " << m_core.m_name << " " << op.addr << " " << op.optype << " " << op.data << std::endl;
    switch(op.optype) {
      case MemoryOperation::READ_BYTE:
        op.return_value(co_await read_byte_internal(op.addr));
        break;
      case MemoryOperation::READ_HALFWORD:{
          out = co_await m_bus_interface.read_halfword(op.addr);
          op.return_value(out);
        } break;
      case MemoryOperation::READ_WORD:
        op.return_value(co_await read_word_internal(op.addr));
        break;
      case MemoryOperation::WRITE_BYTE:
        co_await write_byte_internal(op.addr, op.data);
        op.return_void();
        break;
      case MemoryOperation::WRITE_HALFWORD:
        co_await write_halfword_internal(op.addr, op.data);
        op.return_void();
        break;
      case MemoryOperation::WRITE_WORD:
        co_await write_word_internal(op.addr, op.data);
        op.return_void();
        break;
    }
  }
}

Awaitable<uint8_t> ARMv6MCore::MPU::read_byte_internal(uint32_t addr)
{
  // std::cout << "MPU::read_byte(" << std::hex << addr << std::dec << ")" << std::endl;
  co_return co_await m_bus_interface.read_byte(addr);
}

Awaitable<uint16_t> ARMv6MCore::MPU::read_halfword_internal(uint32_t addr)
{
  uint32_t out;
  out = co_await m_bus_interface.read_halfword(addr);
  // std::cout << "MPU::read_halfword(" << std::hex << addr << std::dec << ") " << out << "  " << m_core.m_name << std::endl;
  co_return out;
}

Awaitable<uint32_t> ARMv6MCore::MPU::read_word_internal(uint32_t addr)
{
  uint32_t out;
  if ((addr & 0xe000'0000) == 0xe000'0000) {
    m_core.m_ppb.read_word(addr, out);
  } else {
    out = co_await m_bus_interface.read_word(addr);
  }
  // std::cout << "MPU::read_word(" << std::hex << addr << std::dec << ") " << out << "  " << m_core.m_name << std::endl;
  co_return out;
}

Awaitable<void> ARMv6MCore::MPU::write_byte_internal(uint32_t addr, uint8_t data)
{
  co_await m_bus_interface.write_byte(addr, data);
}

Awaitable<void> ARMv6MCore::MPU::write_halfword_internal(uint32_t addr, uint16_t data)
{
  co_await m_bus_interface.write_halfword(addr, data);
}

Awaitable<void> ARMv6MCore::MPU::write_word_internal(uint32_t addr, uint32_t data)
{
  // std::cout << "MPU::write_word(" << std::hex << addr << ", " << data << std::dec << ") " << m_core.m_name << std::endl;
  if ((addr & 0xe000'0000) == 0xe000'0000) {
    m_core.m_ppb.write_word(addr, data);
  } else {
    co_await m_bus_interface.write_word(addr, data);
  }
}

