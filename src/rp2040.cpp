#include "rp2040.hpp"
#include "rp2040_bootloader.hpp"
#include <array>
#include <async.hpp>
#include "armv6m/exception.hpp"
#include <iostream>
#include <iomanip>

using namespace RP2040;

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
  // m_
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
  while (ticks++ < 100000) {
    std::cout << "\nTICK " << ticks << std::endl;
    m_cores[0].tick();
    // m_cores[1].tick();
    // m_dma.tick();
    m_ahb.tick();
    // AHBLite.tick();
    // -APB.tick()
    // peripherals.tick()
    m_XIP.tick();
    m_SSI.tick();
  }
}

std::tuple<RP2040::RP2040::AHB::BusDevice, uint32_t> RP2040::RP2040::AHB::lookupBusDeviceAddress(uint32_t addr) const
{
  // std::cout << "lookupBusDeviceAddress(" << std::hex << std::setprecision(8) << addr << std::dec << ")" << std::endl;
  switch(addr & 0xf000'0000) {
    case 0x0000'0000: 
      if (addr < 0x0004'0000) 
        return {BusDevice::ROM, addr&0x0000'ffff}; 
      break;
    case 0x1000'0000:
      return {BusDevice::XIP, addr&0x00ff'ffff};
    case 0x2000'0000:
    {
      constexpr const auto srams = 
        std::array<BusDevice, 6>{
          BusDevice::SRAM0,
          BusDevice::SRAM1,
          BusDevice::SRAM2,
          BusDevice::SRAM3, 
          BusDevice::SRAM4, 
          BusDevice::SRAM5, 
          };
      if (addr < 0x2004'0000) {
        return {srams[(addr >> 2) & 0b11], (addr>>2) & 0x0000'fffc | addr & 0x3};
      } else if (addr < 0x2004'2000) {
        return {srams[4+(addr>>12) & 0b11], addr & 0x0000'0fff};
      } else if (addr >= 0x2100'0000) {
        return {srams[(addr>>16) & 0b11], addr & 0x0000'ffff};
      }
      break;
    }
    case 0x4000'0000:
      return {BusDevice::APB, addr&0x00ff'ffff};
    case 0x5000'0000:
      return {BusDevice::AHBLITE, addr&0x00ff'ffff};
  }
  throw ARMv6M::BusFault();
}

Awaitable<uint8_t> RP2040::RP2040::AHB::read_byte(uint32_t addr) 
{
  //queue read
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  co_await registerBusOp(dev);
  uint8_t out;
  switch(dev) {
    case BusDevice::ROM: out = ((uint8_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset]; break;
    case BusDevice::SRAM0: out = ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset]; break;
    case BusDevice::SRAM1: out = ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset]; break;
    case BusDevice::SRAM2: out = ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset]; break;
    case BusDevice::SRAM3: out = ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset]; break;
    case BusDevice::SRAM4: out = ((uint8_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset]; break;
    case BusDevice::SRAM5: out = ((uint8_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset]; break;
    case BusDevice::APB: out = co_await m_rp2040.m_apb.read_byte(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.m_XIP.read_byte(addr); break;
    default: throw ARMv6M::BusFault{addr};
  }
  co_return out;
}
Awaitable<uint16_t> RP2040::RP2040::AHB::read_halfword(uint32_t addr)
{
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  co_await registerBusOp(dev);
  uint16_t out;
  switch(dev) {
    case BusDevice::ROM: out = ((uint16_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset/2]; break;
    case BusDevice::SRAM0: out = ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset/2]; break;
    case BusDevice::SRAM1: out = ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset/2]; break;
    case BusDevice::SRAM2: out = ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset/2]; break;
    case BusDevice::SRAM3: out = ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset/2]; break;
    case BusDevice::SRAM4: out = ((uint16_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset/2]; break;
    case BusDevice::SRAM5: out = ((uint16_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset/2]; break;
    case BusDevice::APB: out = co_await m_rp2040.m_apb.read_halfword(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.m_XIP.read_halfword(addr); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "read_halfword(" << std::hex << addr << std::dec << ") completed" << std::endl;
  co_return out;
}
Awaitable<uint32_t> RP2040::RP2040::AHB::read_word(uint32_t addr)
{
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  co_await registerBusOp(dev);
  uint32_t out;
  switch(dev) {
    case BusDevice::ROM: out = ((uint32_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset/4]; break;
    case BusDevice::SRAM0: out = ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset/4]; break;
    case BusDevice::SRAM1: out = ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset/4]; break;
    case BusDevice::SRAM2: out = ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset/4]; break;
    case BusDevice::SRAM3: out = ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset/4]; break;
    case BusDevice::SRAM4: out = ((uint32_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset/4]; break;
    case BusDevice::SRAM5: out = ((uint32_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset/4]; break;
    case BusDevice::APB: out = co_await m_rp2040.m_apb.read_word(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.m_XIP.read_word(addr); break;
    default: throw ARMv6M::BusFault{};
  }
  std::cout << "read_word(" << std::hex << addr << std::dec << ") completed " << out << std::endl;
  co_return out;
}

Awaitable<void> RP2040::RP2040::AHB::write_byte(uint32_t addr, uint8_t val)
{
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  co_await registerBusOp(dev);
  switch(dev) {
    case BusDevice::ROM: ((uint8_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset] = val; break;
    case BusDevice::SRAM0: ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset] = val; break;
    case BusDevice::SRAM1: ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset] = val; break;
    case BusDevice::SRAM2: ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset] = val; break;
    case BusDevice::SRAM3: ((uint8_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset] = val; break;
    case BusDevice::SRAM4: ((uint8_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset] = val; break;
    case BusDevice::SRAM5: ((uint8_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset] = val; break;
    case BusDevice::APB: co_await m_rp2040.m_apb.write_byte(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
}
Awaitable<void> RP2040::RP2040::AHB::write_halfword(uint32_t addr, uint16_t val)
{
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  co_await registerBusOp(dev);
  switch(dev) {
    case BusDevice::ROM: ((uint16_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset/2] = val; break;
    case BusDevice::SRAM0: ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset/2] = val; break;
    case BusDevice::SRAM1: ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset/2] = val; break;
    case BusDevice::SRAM2: ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset/2] = val; break;
    case BusDevice::SRAM3: ((uint16_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset/2] = val; break;
    case BusDevice::SRAM4: ((uint16_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset/2] = val; break;
    case BusDevice::SRAM5: ((uint16_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset/2] = val; break;
    case BusDevice::APB: co_await m_rp2040.m_apb.write_halfword(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
}
Awaitable<void> RP2040::RP2040::AHB::write_word(uint32_t addr, uint32_t val)
{
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  co_await registerBusOp(dev);
  std::cout << "writing word to " << std::hex << addr << std::dec << " dev " << (int)dev << " offset " << offset << " val " << val << std::endl;
  switch(dev) {
    case BusDevice::ROM: ((uint32_t*) /* [8192] */(m_rp2040.m_ROM.begin()))[offset/4] = val; break;
    case BusDevice::SRAM0: ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM0.begin()))[offset/4] = val; break;
    case BusDevice::SRAM1: ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM1.begin()))[offset/4] = val; break;
    case BusDevice::SRAM2: ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM2.begin()))[offset/4] = val; break;
    case BusDevice::SRAM3: ((uint32_t*) /* [32768] */(m_rp2040.m_SRAM3.begin()))[offset/4] = val; break;
    case BusDevice::SRAM4: ((uint32_t*) /* [2048] */(m_rp2040.m_SRAM4.begin()))[offset/4] = val; break;
    case BusDevice::SRAM5: ((uint32_t*) /* [2048] */(m_rp2040.m_SRAM5.begin()))[offset/4] = val; break;
    case BusDevice::APB: co_await m_rp2040.m_apb.write_word(addr, val); break;
    case BusDevice::XIP: co_await m_rp2040.m_XIP.write_word(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "write_word(" << std::hex << addr << std::dec << ") completed" << std::endl;
}

Awaitable<uint8_t> RP2040::RP2040::CoreBus::read_byte(uint32_t addr)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint8_t val;
    m_ioport.read_byte(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_byte(addr);
}
Awaitable<uint16_t> RP2040::RP2040::CoreBus::read_halfword(uint32_t addr)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint16_t val;
    m_ioport.read_halfword(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_halfword(addr);
}
Awaitable<uint32_t> RP2040::RP2040::CoreBus::read_word(uint32_t addr)
{
  // std::cout << "CoreBus::read_word " << std::hex << addr << std::dec << std::endl;
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    uint32_t val;
    m_ioport.read_word(addr, val);
    co_return val;
  }
  co_return co_await m_ahb.read_word(addr);
}

Awaitable<void> RP2040::RP2040::CoreBus::write_byte(uint32_t addr, uint8_t val)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_byte(addr, val);
  } else {
    co_await m_ahb.write_byte(addr, val);
  }
}
Awaitable<void> RP2040::RP2040::CoreBus::write_halfword(uint32_t addr, uint16_t val)
{
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_halfword(addr, val);
  } else {
    co_await m_ahb.write_halfword(addr, val);
  }
}
Awaitable<void> RP2040::RP2040::CoreBus::write_word(uint32_t addr, uint32_t val)
{
  // std::cout << "CoreBus::write_word " << std::hex << addr << std::dec << "\n";
  if ((addr & 0xf000'0000) == 0xd000'0000) {
    m_ioport.write_word(addr, val);
  } else {
    co_await m_ahb.write_word(addr, val);
  }
}

void RP2040::RP2040::AHB::tick()
{
  for(int i = 0; i < BusDevice::DEVICE_MAX; i++) {
    auto& [q] = m_busOps[i];
    if (q.empty())
      continue;
    auto h = q.front();
    h.resume();
    q.pop();
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
  switch(addr) {
    case 0xd000'0000: out = m_cpuid; break;
    case 0xd000'0008: out = 0x0000'0002; break; // force CS high to enable flash boot
    case 0xd000'0050: out = m_tx_fifo.status_send() | m_rx_fifo.status_recv(); break;
    // case 0xd000'0054: out = ; break;
    case 0xd000'0058: out = m_rx_fifo.recv(); break;
    case 0xd000'0070: out = m_divider.get_quotient(); break;
    case 0xd000'0074: out = m_divider.get_remainder(); break;
    case 0xd000'0078: out = m_divider.get_status(); break;
    default: throw ARMv6M::BusFault{addr};
  }
  return PortState::SUCCESS; 
}
PortState RP2040::RP2040::IOPort::write_byte(uint32_t addr, uint8_t in){ throw ARMv6M::BusFault(); }
PortState RP2040::RP2040::IOPort::write_halfword(uint32_t addr, uint16_t in){ throw ARMv6M::BusFault(); }
PortState RP2040::RP2040::IOPort::write_word(uint32_t addr, uint32_t in){ 
  // std::cout << "IOPort::write_word(" << std::hex << addr << ", " << in << std::dec << ")" << std::endl;
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
    default: throw ARMv6M::BusFault{addr};
  }
  return PortState::SUCCESS; 
}