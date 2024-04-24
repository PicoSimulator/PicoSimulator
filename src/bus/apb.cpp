#include "rp2040.hpp"

Awaitable<uint8_t> RP2040::APB::read_byte(uint32_t addr)
{
  uint8_t out = co_await read_word(addr);
  co_return out;
}
Awaitable<uint16_t> RP2040::APB::read_halfword(uint32_t addr)
{
  uint16_t out = co_await read_word(addr);
  co_return out;
}
Awaitable<uint32_t> RP2040::APB::read_word(uint32_t addr)
{
  co_return 0;
}
Awaitable<void> RP2040::APB::write_byte(uint32_t addr, uint8_t in)
{
  co_await write_word(addr, in | (in << 8) | (in << 16) | (in << 24));
}
Awaitable<void> RP2040::APB::write_halfword(uint32_t addr, uint16_t in)
{
  co_await write_word(addr, in | (in << 16));
}
Awaitable<void> RP2040::APB::write_word(uint32_t addr, uint32_t in)
{
  // co_await m_rp2040.m_corebus.write_word(addr, in);
  co_return;
}