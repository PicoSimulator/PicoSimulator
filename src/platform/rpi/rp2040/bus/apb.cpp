#include "platform/rpi/rp2040/rp2040.hpp"
#include "arch/arm/armv6m/exception.hpp"

using namespace RP2040::Bus;

#define _(...)

#define APB_PERIPHERALS(o) \
  o(0x0000'4000, m_syscfg, SYNC, NOTICK) \
  o(0x0000'8000, m_clocks, SYNC, NOTICK) \
  o(0x0000'c000, m_resets, SYNC, NOTICK) \
  o(0x0001'0000, m_psm, SYNC, NOTICK)\
  o(0x0001'4000, m_io_b0, SYNC, NOTICK) \
  o(0x0001'8000, m_io_qspi, SYNC, NOTICK) \
  o(0x0001'c000, m_pads_b0, SYNC, NOTICK) \
  o(0x0002'0000, m_pads_qspi, SYNC, NOTICK) \
  o(0x0002'4000, m_xosc, SYNC, NOTICK) \
  o(0x0002'8000, m_pll_sys, SYNC, NOTICK) \
  o(0x0002'c000, m_pll_usb, SYNC, NOTICK) \
  o(0x0003'4000, m_uart0, SYNC, NOTICK) \
  o(0x0003'8000, m_uart1, SYNC, NOTICK) \
  o(0x0003'c000, m_spi0, SYNC, NOTICK) \
  o(0x0004'0000, m_spi1, SYNC, NOTICK) \
  _(0x0004'4000, m_i2c0, SYNC, NOTICK) \
  _(0x0004'8000, m_i2c1, SYNC, NOTICK) \
  o(0x0005'4000, m_timer, SYNC, NOTICK) \
  o(0x0005'8000, m_watchdog, SYNC, NOTICK) \
  o(0x0005'c000, m_rtc, SYNC, NOTICK) \
  o(0x0006'0000, m_rosc, SYNC, NOTICK) \
  o(0x0006'4000, m_vreg, SYNC, NOTICK) \
  o(0x0006'c000, m_tbman, SYNC, NOTICK) \

void APB::tick()
{
  #define ENUM_PERIPHERAL(peri_addr, name, _, tick) tick (name)
  #define TICK(name) name.tick();
  #define NOTICK(name) 
    APB_PERIPHERALS(ENUM_PERIPHERAL)
  #undef ENUM_PERIPHERAL
  #undef TICK
  #undef NOTICK
}

Task APB::bus_task()
{
  while(true) {
    auto &op = co_await next_op();
    switch(op.optype) {
      case MemoryOperation::READ_BYTE:
        // m_op->return_value(co_await read_byte_internal(m_op->addr));
        // break;
      case MemoryOperation::READ_HALFWORD:
        // m_op->return_value(co_await read_halfword_internal(m_op->addr));
        // break;
      case MemoryOperation::READ_WORD:
      {
        #define ENUM_PERIPHERAL(peri_addr, name, synch, _) case peri_addr: synch (name.read_word, op.addr, out); break;
        #define SYNC(fun, addr, out) fun(addr, out)
        #define ASYNC(fun, addr, out) out = co_await fun(addr) 
        
        uint32_t out;
        switch(op.addr & 0x00ff'c000) {
          APB_PERIPHERALS(ENUM_PERIPHERAL)
          default:
            throw ARMv6M::BusFault{op.addr};
        }
        #undef ENUM_PERIPHERAL
        #undef SYNC
        #undef ASYNC
        m_op->return_value(out);
      } break;
      case MemoryOperation::WRITE_BYTE:
        co_await write_byte_internal(m_op->addr, m_op->data);
        m_op->return_void();
        break;
      case MemoryOperation::WRITE_HALFWORD:
        co_await write_halfword_internal(m_op->addr, m_op->data);
        m_op->return_void();
        break;
      case MemoryOperation::WRITE_WORD:
        co_await write_word_internal(m_op->addr, m_op->data);
        m_op->return_void();
        break;
    }
  }
}

Awaitable<uint8_t> APB::read_byte_internal(uint32_t addr)
{
  uint8_t out = co_await read_word_internal(addr);
  co_return out;
}
Awaitable<uint16_t> APB::read_halfword_internal(uint32_t addr)
{
  uint16_t out = co_await read_word_internal(addr);
  co_return out;
}
Awaitable<uint32_t> APB::read_word_internal(uint32_t addr)
{
  #define ENUM_PERIPHERAL(peri_addr, name, synch, _) case peri_addr: synch (name.read_word, addr, out); break;
  #define SYNC(fun, addr, out) fun(addr, out)
  #define ASYNC(fun, addr, out) out = co_await fun(addr) 
  
  uint32_t out;
  switch(addr & 0x00ff'c000) {
    APB_PERIPHERALS(ENUM_PERIPHERAL)
    default:
      throw ARMv6M::BusFault{addr};
  }
  #undef ENUM_PERIPHERAL
  #undef SYNC
  #undef ASYNC
  co_return out;
}
Awaitable<void> APB::write_byte_internal(uint32_t addr, uint8_t in)
{
  co_await write_word_internal(addr, in | (in << 8) | (in << 16) | (in << 24));
}
Awaitable<void> APB::write_halfword_internal(uint32_t addr, uint16_t in)
{
  co_await write_word_internal(addr, in | (in << 16));
}
Awaitable<void> APB::write_word_internal(uint32_t addr, uint32_t in)
{
  #define ENUM_PERIPHERAL(peri_addr, name, synch, _) case peri_addr: synch (name.write_word, addr, in); break;
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