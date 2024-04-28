#include "rp2040.hpp"
#include "armv6m/exception.hpp"

#define APB_PERIPHERALS(o) \
  o(0x0000'4000, m_syscfg, SYNC, NOTICK) \
  o(0x0000'8000, m_clocks, SYNC, NOTICK) \
  o(0x0000'c000, m_resets, SYNC, NOTICK) \
  o(0x0001'8000, m_io_qspi, SYNC, NOTICK) \
  o(0x0001'c000, m_pads_b0, SYNC, NOTICK) \
  o(0x0002'0000, m_pads_qspi, SYNC, NOTICK) \
  o(0x0005'8000, m_watchdog, SYNC, NOTICK) \
  o(0x0006'4000, m_vreg, SYNC, NOTICK) \
  o(0x0006'c000, m_tbman, SYNC, NOTICK) \

void RP2040::APB::tick()
{
  #define ENUM_PERIPHERAL(peri_addr, name, _, tick) tick (name)
  #define TICK(name) name.tick();
  #define NOTICK(name) 
    APB_PERIPHERALS(ENUM_PERIPHERAL)
  #undef ENUM_PERIPHERAL
  #undef TICK
  #undef NOTICK
}

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
  #define ENUM_PERIPHERAL(peri_addr, name, synch, _) case peri_addr: synch (name.read_word, addr, out); break;
  #define SYNC(fun, addr, out) fun(addr, out)
  #define ASYNC(fun, addr, out) out = co_await fun(addr) 
  
  uint32_t out;
  switch(addr & 0x000f'c000) {
    APB_PERIPHERALS(ENUM_PERIPHERAL)
    default:
      throw ARMv6M::BusFault{addr};
  }
  #undef ENUM_PERIPHERAL
  #undef SYNC
  #undef ASYNC
  co_return out;
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
  #define ENUM_PERIPHERAL(peri_addr, name, synch, _) case peri_addr: synch (name.read_word, addr, in); break;
  #define SYNC(fun, addr, in) fun(addr, in)
  #define ASYNC(fun, addr, in) co_await fun(addr, in) 
  
  switch(addr & 0x000f'c000) {
    APB_PERIPHERALS(ENUM_PERIPHERAL)
    default:
      throw ARMv6M::BusFault{addr};
  }
  #undef ENUM_PERIPHERAL
  #undef SYNC
  #undef ASYNC
  co_return;
}