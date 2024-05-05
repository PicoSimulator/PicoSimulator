#include "rp2040.hpp"
#include "rp2040_bootloader.hpp"
#include <array>
#include <async.hpp>
#include "armv6m/exception.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

RP2040::RP2040::RP2040()
: m_XIP{m_SSI}
, m_SSI{*this}
, m_ioports{{0, m_fifo_01, m_fifo_10}, {1, m_fifo_10, m_fifo_01}}
, m_ROM{load_bootloader(BootloaderVersion::B2)}
, m_bus_masters{m_cores[0].run(), m_cores[1].run()}
, m_apb{m_resets, m_vreg, m_clocks, m_syscfg}
, m_ahb{*this}
, m_core_bus{{m_ioports[0], m_ahb}, {m_ioports[1], m_ahb}}
, m_cores{{m_core_bus[0], "core-0"}, {m_core_bus[1], "core-1"}}
{
  clk_sys.sink_add(m_cores[0]);
  clk_sys.sink_add(m_cores[1]);
  clk_sys.sink_add(m_ahb);
  // clk_sys.sink_add(m_dma);
  clk_sys.sink_add(m_XIP);
  clk_sys.sink_add(m_SSI);
  clk_ref.sink_add(m_apb.timer());
  clk_peri.sink_add(m_apb.uart0());
}

void RP2040::RP2040::reset()
{
  std::cout << "RP2040::reset()" << std::endl;
  m_cores[0].reset();
  m_cores[1].reset();
}

void RP2040::RP2040::run()
{
  int ticks = 0;
  while (ticks++ < 1000000) {
    // std::cout << "\nTICK " << ticks << std::endl;
    clk_sys.tick();
    clk_ref.tick();
    clk_peri.tick();
  }
}

void RP2040::RP2040::load_binary(const std::string &path)
{
  std::cout << "RP2040::load_binary(" << path << ")" << std::endl;
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buf;
  buf.resize(size);

  if (file.read((char*)buf.data(), size))
  {
      /* worked! */
  } else {
      /* failed! */
      std::terminate();
  }

  m_XIP.load_binary_data(buf);
  m_SSI.spidev().load_binary_data(buf);
}

UART &RP2040::RP2040::UART0()
{
  return m_apb.uart0();
}

Task RP2040::RP2040::CoreBus::bus_task()
{
  while(true) {
    auto &op = co_await next_op();
    switch(op.optype) {
      case MemoryOperation::READ_BYTE:
        co_await op.return_value(co_await read_byte_internal(op.addr));
        break;
      case MemoryOperation::READ_HALFWORD:{
          uint32_t addr = op.addr;
          if ((addr & 0xf000'0000) == 0xd000'0000) {
            uint16_t val;
            m_ioport.read_halfword(addr, val);
            co_await op.return_value(val);
          } else {
            co_await op.return_value(co_await m_ahb.read_halfword(addr));
          }
        }
        break;
      case MemoryOperation::READ_WORD:
        co_await op.return_value(co_await read_word_internal(op.addr));
        break;
      case MemoryOperation::WRITE_BYTE:
        co_await write_byte_internal(op.addr, op.data);
        co_await op.return_void();
        break;
      case MemoryOperation::WRITE_HALFWORD:
        co_await write_halfword_internal(op.addr, op.data);
        co_await op.return_void();
        break;
      case MemoryOperation::WRITE_WORD:
        // std::cout << "CoreBus::bus_task WRITE_WORD " << std::hex << op.addr << ":" << op.data << std::dec << std::endl;
        co_await write_word_internal(op.addr, op.data);
        co_await op.return_void();
        break;
    }
  }

}

Awaitable<uint8_t> RP2040::RP2040::CoreBus::read_byte_internal(uint32_t addr)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint8_t val;
    m_ioport.read_byte(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_byte(addr);
}
Awaitable<uint16_t> RP2040::RP2040::CoreBus::read_halfword_internal(uint32_t addr)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint16_t val;
    m_ioport.read_halfword(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_halfword(addr);
}
Awaitable<uint32_t> RP2040::RP2040::CoreBus::read_word_internal(uint32_t addr)
{
  // std::cout << "CoreBus::read_word " << std::hex << addr << std::dec << std::endl;
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint32_t val;
    m_ioport.read_word(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_word(addr);
}

Awaitable<void> RP2040::RP2040::CoreBus::write_byte_internal(uint32_t addr, uint8_t val)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_byte(addr, val);
  } else {
    co_await m_ahb.write_byte(addr, val);
  }
}
Awaitable<void> RP2040::RP2040::CoreBus::write_halfword_internal(uint32_t addr, uint16_t val)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_halfword(addr, val);
  } else {
    co_await m_ahb.write_halfword(addr, val);
  }
}
Awaitable<void> RP2040::RP2040::CoreBus::write_word_internal(uint32_t addr, uint32_t val)
{
  // std::cout << "CoreBus::write_word " << std::hex << addr << std::dec << "\n";
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_word(addr, val);
  } else {
    co_await m_ahb.write_word(addr, val);
  }
}


PortState RP2040::RP2040::IOPort::read_byte(uint32_t addr, uint8_t &out){ 
  uint32_t out2;
  PortState ret = read_word(addr, out2);
  out = out2;
  return ret;
}
PortState RP2040::RP2040::IOPort::read_halfword(uint32_t addr, uint16_t &out){ 
  uint32_t out2;
  PortState ret = read_word(addr, out2);
  out = out2;
  return ret;
}
PortState RP2040::RP2040::IOPort::read_word(uint32_t addr, uint32_t &out){ 
  // std::cout << "IOPort::read_word(" << std::hex << addr << std::dec << ")" << std::endl;
  #define SPINLOCKS_EVAL(num) case (0xd000'0100 + 4*num): out = m_spinlocks.try_lock(num); break;
  switch(addr) {
    case 0xd000'0000: out = m_cpuid; break;
    case 0xd000'0008: out = 0x0000'0002; break; // force CS high to enable flash boot
    case 0xd000'0050: out = m_tx_fifo.status_send() | m_rx_fifo.status_recv(); break;
    // case 0xd000'0054: out = ; break;
    case 0xd000'0058: out = m_rx_fifo.recv(); break;
    case 0xd000'0060: out = m_divider.get_dividend(); break;
    case 0xd000'0064: out = m_divider.get_divisor(); break;
    case 0xd000'0068: out = m_divider.get_dividend(); break;
    case 0xd000'006c: out = m_divider.get_divisor(); break;
    case 0xd000'0070: out = m_divider.get_quotient(); break;
    case 0xd000'0074: out = m_divider.get_remainder(); break;
    case 0xd000'0078: out = m_divider.get_status(); break;
    ENUM_SPINLOCKS(SPINLOCKS_EVAL)
    default: throw ARMv6M::BusFault{addr};
  }
  #undef SPINLOCKS_EVAL
  return PortState::SUCCESS; 
}
PortState RP2040::RP2040::IOPort::write_byte(uint32_t addr, uint8_t in){ throw ARMv6M::BusFault(); }
PortState RP2040::RP2040::IOPort::write_halfword(uint32_t addr, uint16_t in){ throw ARMv6M::BusFault(); }
PortState RP2040::RP2040::IOPort::write_word(uint32_t addr, uint32_t in){ 
  // std::cout << "IOPort::write_word(" << std::hex << addr << ", " << in << std::dec << ")" << std::endl;
  #define SPINLOCKS_EVAL(num) case (0xd000'0100 + 4*num): m_spinlocks.unlock(num); break;
  switch(addr) {
    case 0xd000'0000: break;
    case 0xd000'0050: 
      if (in & (1<<2)) m_tx_fifo.clear_send();
      if (in & (1<<3)) m_rx_fifo.clear_recv();
      break;
    case 0xd000'0054: m_tx_fifo.send(in); break;
    case 0xd000'0060: m_divider.set_udividend(in); break;
    case 0xd000'0064: m_divider.set_udivisor(in); break;
    case 0xd000'0068: m_divider.set_sdividend(in); break;
    case 0xd000'006c: m_divider.set_sdivisor(in); break;
    case 0xd000'0070: m_divider.set_quotient(in); break;
    case 0xd000'0074: m_divider.set_remainder(in); break;
    ENUM_SPINLOCKS(SPINLOCKS_EVAL)

    default: throw ARMv6M::BusFault{addr};
  }
  #undef SPINLOCKS_EVAL
  return PortState::SUCCESS; 
}