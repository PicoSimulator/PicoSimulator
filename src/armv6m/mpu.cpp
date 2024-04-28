#include "armv6m/core.hpp"

using namespace ARMv6M;

// currently just a transparent interface to the next bus layer

ARMv6MCore::MPU::MPU(IAsyncReadWritePort<uint32_t> &bus, ARMv6MCore &core) 
: m_bus_interface{bus}
, m_core{core}
{
}

Awaitable<uint8_t> ARMv6MCore::MPU::read_byte(uint32_t addr)
{
  std::cout << "MPU::read_byte(" << std::hex << addr << std::dec << ")" << std::endl;
  co_return co_await m_bus_interface.read_byte(addr);
}

Awaitable<uint16_t> ARMv6MCore::MPU::read_halfword(uint32_t addr)
{
  uint32_t out;
  out = co_await m_bus_interface.read_halfword(addr);
  std::cout << "MPU::read_halfword(" << std::hex << addr << std::dec << ") " << out << "  " << m_core.m_name << std::endl;
  co_return out;
}

Awaitable<uint32_t> ARMv6MCore::MPU::read_word(uint32_t addr)
{
  uint32_t out;
  if ((addr & 0xe000'0000) == 0xe000'0000) {
    m_core.m_ppb.read_word(addr, out);
  } else {
    out = co_await m_bus_interface.read_word(addr);
  }
  std::cout << "MPU::read_word(" << std::hex << addr << std::dec << ") " << out << "  " << m_core.m_name << std::endl;
  co_return out;
}

Awaitable<void> ARMv6MCore::MPU::write_byte(uint32_t addr, uint8_t data)
{
  co_await m_bus_interface.write_byte(addr, data);
}

Awaitable<void> ARMv6MCore::MPU::write_halfword(uint32_t addr, uint16_t data)
{
  co_await m_bus_interface.write_halfword(addr, data);
}

Awaitable<void> ARMv6MCore::MPU::write_word(uint32_t addr, uint32_t data)
{
  std::cout << "MPU::write_word(" << std::hex << addr << ", " << data << std::dec << ") " << m_core.m_name << std::endl;
  if ((addr & 0xe000'0000) == 0xe000'0000) {
    m_core.m_ppb.write_word(addr, data);
  } else {
    co_await m_bus_interface.write_word(addr, data);
  }
}

