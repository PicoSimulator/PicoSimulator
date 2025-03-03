#include "platform/rpi/rp2040/bus/ahb.hpp"
#include "arch/arm/armv6m/exception.hpp"
#include "platform/rpi/rp2040/rp2040.hpp"

#include <cassert>

#define byte_array_read_as_byte(arr, index)     ({ uint32_t i = (index); (uint8_t{(arr)[(i)]}) ; })
#define byte_array_read_as_halfword(arr, index) ({ uint32_t i = (index); (uint16_t{(arr)[(i)]} | (uint16_t{(arr)[(i)+1]} << 8)) ; })
#define byte_array_read_as_word(arr, index)     ({ uint32_t i = (index); (uint32_t{(arr)[(i)]} | (uint32_t{(arr)[(i)+1]} << 8) | (uint32_t{(arr)[(i)+2]} << 16) | (uint32_t{(arr)[(i)+3]} << 24)) ; })

#define byte_array_write_as_byte(arr, index, val)     ({ uint32_t i = (index); uint8_t v = (val); (arr)[(i)] = v; v; })
#define byte_array_write_as_halfword(arr, index, val) ({ uint32_t i = (index); uint16_t v = (val); (arr)[(i)] = v & 0xff; (arr)[(i)+1] = (v >> 8) & 0xff; v; })
#define byte_array_write_as_word(arr, index, val)     ({ uint32_t i = (index); uint32_t v = (val); (arr)[(i)] = v & 0xff; (arr)[(i)+1] = (v >> 8) & 0xff; (arr)[(i)+2] = (v >> 16) & 0xff; (arr)[(i)+3] = (v >> 24) & 0xff; v; })

using namespace RP2040::Bus;

void AHB::tick()
{
  // std::cout << "AHB::tick()" << std::endl;
  // check if any of the devices on AHB have operations outstanding
  for(int i = 0; i < BusDevice::DEVICE_MAX; i++) {
    auto& [q, busy] = m_busOps[i];
    if (q.empty() || busy)
      continue;
    auto h = q.top();
    // std::cout << "Mem Op Starting AHB: " << std::dec << i << std::endl;
    busy = true;
    q.pop();
    for(int i = 0; i < 4; i++) {
      if (m_ops[i] != nullptr) continue;
      m_ops[i] = &(h.get());

      m_runners[i].resume();
      break;
    }
  }
}
bool AHB::register_op(MemoryOperation &op)
{
  // std::cout << "register_op" << std::endl;
  // m_port_address[op.]
  auto [dev, addr] = lookupBusDeviceAddress(op.addr);
  m_port_address[op.upstream_id] = addr;

  auto &[q, busy] = m_busOps[dev];
  q.push(op);
  return false;
}

// void AHB::deregister_op(MemoryOperation &op)
// {
//   // std::cout << "deregister_op" << std::endl;
//   auto [dev, addr] = lookupBusDeviceAddress(op.addr);
//   auto &[q, busy] = m_busOps[dev];
//   // if (q.empty())
//   busy = false;
//   for (int i = 0; i < 4; i++) {
//     if (m_ops[i] == &op) {
//       m_ops[i] = nullptr;
//       return;
//     }
//   }
//   assert(false);
// }

Task AHB::bus_task(uint32_t id)
{
  while(true){
    auto &op = co_await next_op(id);
    // std::cout << "AHB::bus_task(" << id << ")" << std::endl;
    auto val = lookupBusDeviceAddress(op.addr);
    auto dev = std::get<0>(val);
    auto offset = std::get<1>(val);
    // std::cout << dev << "," << offset << std::endl;
    switch(op.optype) {
      case MemoryOperation::OpType::READ_BYTE: 
        op.return_value(co_await read_byte_internal(op.addr));
        break;
      case MemoryOperation::OpType::READ_HALFWORD: {
          uint16_t out;
          switch(dev) {
            case BusDevice::ROM: out = /* [8192] */ byte_array_read_as_halfword(m_rp2040.ROM(), offset); break;
            case BusDevice::SRAM0: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM0(), offset); break;
            case BusDevice::SRAM1: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM1(), offset); break;
            case BusDevice::SRAM2: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM2(), offset); break;
            case BusDevice::SRAM3: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM3(), offset); break;
            case BusDevice::SRAM4: out = /* [2048] */ byte_array_read_as_halfword(m_rp2040.SRAM4(), offset); break;
            case BusDevice::SRAM5: out = /* [2048] */ byte_array_read_as_halfword(m_rp2040.SRAM5(), offset); break;
            case BusDevice::APB: out = co_await m_rp2040.APB().read_halfword(op.addr); break;
            case BusDevice::XIP: out = co_await m_rp2040.XIP().read_halfword(op.addr); break;
            case BusDevice::AHBLITE: m_rp2040.AHBLite().read_halfword(op.addr, out); break;
            default: throw ARMv6M::BusFault{op.addr};
          }
          // std::cout << "read_halfword(" << std::hex << addr << std::dec << ") completed" << std::endl;
          op.return_value(out);
      } break;
      case MemoryOperation::OpType::READ_WORD: {
        uint32_t out;
        switch(dev) {
          case BusDevice::ROM: out = /* [8192] */ byte_array_read_as_word(m_rp2040.ROM(), offset); break;
          case BusDevice::SRAM0: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM0(), offset); break;
          case BusDevice::SRAM1: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM1(), offset); break;
          case BusDevice::SRAM2: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM2(), offset); break;
          case BusDevice::SRAM3: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM3(), offset); break;
          case BusDevice::SRAM4: out = /* [2048] */ byte_array_read_as_word(m_rp2040.SRAM4(), offset); break;
          case BusDevice::SRAM5: out = /* [2048] */ byte_array_read_as_word(m_rp2040.SRAM5(), offset); break;
          case BusDevice::APB: out = co_await m_rp2040.APB().read_word(op.addr); break;
          case BusDevice::XIP: out = co_await m_rp2040.XIP().read_word(op.addr); break;
          case BusDevice::AHBLITE: m_rp2040.AHBLite().read_word(op.addr, out); break;
          default: throw ARMv6M::BusFault{op.addr};
        }        
        op.return_value(out);
      } break;
      case MemoryOperation::OpType::WRITE_BYTE: 
        co_await write_byte_internal(op.addr, op.data);
        op.return_void();
        break;
      case MemoryOperation::OpType::WRITE_HALFWORD: 
        co_await write_halfword_internal(op.addr, op.data);
        op.return_void();
        break;
      case MemoryOperation::OpType::WRITE_WORD: {
        auto devoffset = lookupBusDeviceAddress(op.addr);
        auto dev = std::get<0>(devoffset);
        auto offset = std::get<1>(devoffset);
        // std::cout << "writing word to " << std::hex << op.addr << std::dec << " dev " << (int)dev << " offset " << offset << " val " << op.data << std::endl;
        switch(dev) {
          case BusDevice::ROM: /* [8192] */  break;
          case BusDevice::SRAM0: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM0(), offset, op.data); break;
          case BusDevice::SRAM1: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM1(), offset, op.data); break;
          case BusDevice::SRAM2: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM2(), offset, op.data); break;
          case BusDevice::SRAM3: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM3(), offset, op.data); break;
          case BusDevice::SRAM4: /* [2048] */ byte_array_write_as_word(m_rp2040.SRAM4(), offset, op.data); break;
          case BusDevice::SRAM5: /* [2048] */ byte_array_write_as_word(m_rp2040.SRAM5(), offset, op.data); break;
          case BusDevice::APB: co_await m_rp2040.APB().write_word(op.addr, op.data); break;
          case BusDevice::XIP: co_await m_rp2040.XIP().write_word(op.addr, op.data); break;
          case BusDevice::AHBLITE: m_rp2040.AHBLite().write_word(op.addr, op.data); break;
          default: throw ARMv6M::BusFault{op.addr};
        }
        op.return_void();
      } break;
    }
  }
}

inline std::tuple<AHB::BusDevice, uint32_t> AHB::lookupBusDeviceAddress(uint32_t addr) const
{
  // std::cout << "lookupBusDeviceAddress(" << std::hex << std::setprecision(8) << addr << std::dec << ")" << std::endl;
  switch(addr & 0xf000'0000) {
    case 0x0000'0000: 
      if (addr < 0x0004'0000) 
        return {BusDevice::ROM, addr/* &0x0000'ffff */}; 
      break;
    case 0x1000'0000:
      return {BusDevice::XIP, addr/* &0x00ff'ffff */};
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
        return {srams[(addr >> 2) & 0b11], ((addr>>2) & 0x0000'fffc) | (addr & 0x3)};
      } else if (addr < 0x2004'2000) {
        return {srams[4+((addr>>12) & 1)], addr & 0x0000'0fff};
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
  throw ARMv6M::BusFault(addr);
}

Awaitable<uint8_t> AHB::read_byte_internal(uint32_t addr) 
{
  //queue read
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  uint8_t out;
  switch(dev) {
    case BusDevice::ROM: out = /* [8192] */byte_array_read_as_byte(m_rp2040.ROM(), offset); break;
    case BusDevice::SRAM0: out = /* [32768] */byte_array_read_as_byte(m_rp2040.SRAM0(), offset); break;
    case BusDevice::SRAM1: out = /* [32768] */byte_array_read_as_byte(m_rp2040.SRAM1(), offset); break;
    case BusDevice::SRAM2: out = /* [32768] */byte_array_read_as_byte(m_rp2040.SRAM2(), offset); break;
    case BusDevice::SRAM3: out = /* [32768] */byte_array_read_as_byte(m_rp2040.SRAM3(), offset); break;
    case BusDevice::SRAM4: out = /* [2048] */byte_array_read_as_byte(m_rp2040.SRAM4(), offset); break;
    case BusDevice::SRAM5: out = /* [2048] */byte_array_read_as_byte(m_rp2040.SRAM5(), offset); break;
    case BusDevice::APB: out = co_await m_rp2040.APB().read_byte(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.XIP().read_byte(addr); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().read_byte(addr, out); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "read_byte(" << std::hex << addr << std::dec << ") completed " << uint{out} << std::endl;
  co_return out;
}
Awaitable<uint16_t> AHB::read_halfword_internal(uint32_t addr)
{
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  uint16_t out;
  switch(dev) {
    case BusDevice::ROM: out = /* [8192] */ byte_array_read_as_halfword(m_rp2040.ROM(), offset); break;
    case BusDevice::SRAM0: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM0(), offset); break;
    case BusDevice::SRAM1: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM1(), offset); break;
    case BusDevice::SRAM2: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM2(), offset); break;
    case BusDevice::SRAM3: out = /* [32768] */ byte_array_read_as_halfword(m_rp2040.SRAM3(), offset); break;
    case BusDevice::SRAM4: out = /* [2048] */ byte_array_read_as_halfword(m_rp2040.SRAM4(), offset); break;
    case BusDevice::SRAM5: out = /* [2048] */ byte_array_read_as_halfword(m_rp2040.SRAM5(), offset); break;
    case BusDevice::APB: out = co_await m_rp2040.APB().read_halfword(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.XIP().read_halfword(addr); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().read_halfword(addr, out); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "read_halfword(" << std::hex << addr << std::dec << ") completed" << std::endl;
  co_return out;
}
Awaitable<uint32_t> AHB::read_word_internal(uint32_t addr)
{
  auto val = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(val);
  auto offset = std::get<1>(val);
  uint32_t out;
  switch(dev) {
    case BusDevice::ROM: out = /* [8192] */ byte_array_read_as_word(m_rp2040.ROM(), offset); break;
    case BusDevice::SRAM0: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM0(), offset); break;
    case BusDevice::SRAM1: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM1(), offset); break;
    case BusDevice::SRAM2: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM2(), offset); break;
    case BusDevice::SRAM3: out = /* [32768] */ byte_array_read_as_word(m_rp2040.SRAM3(), offset); break;
    case BusDevice::SRAM4: out = /* [2048] */ byte_array_read_as_word(m_rp2040.SRAM4(), offset); break;
    case BusDevice::SRAM5: out = /* [2048] */ byte_array_read_as_word(m_rp2040.SRAM5(), offset); break;
    case BusDevice::APB: out = co_await m_rp2040.APB().read_word(addr); break;
    case BusDevice::XIP: out = co_await m_rp2040.XIP().read_word(addr); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().read_word(addr, out); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "read_word(" << std::hex << addr << std::dec << ") completed " << out << std::endl;
  co_return out;
}

Awaitable<void> AHB::write_byte_internal(uint32_t addr, uint8_t val)
{
  // std::cout << "AHB::write_byte_internal(" << std::hex << addr << std::dec << ")" << std::endl;
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  uint16_t out;
  switch(dev) {
    case BusDevice::ROM: /* [8192] */  break;
    case BusDevice::SRAM0: /* [32768] */ byte_array_write_as_byte(m_rp2040.SRAM0(), offset, val); break;
    case BusDevice::SRAM1: /* [32768] */ byte_array_write_as_byte(m_rp2040.SRAM1(), offset, val); break;
    case BusDevice::SRAM2: /* [32768] */ byte_array_write_as_byte(m_rp2040.SRAM2(), offset, val); break;
    case BusDevice::SRAM3: /* [32768] */ byte_array_write_as_byte(m_rp2040.SRAM3(), offset, val); break;
    case BusDevice::SRAM4: /* [2048] */ byte_array_write_as_byte(m_rp2040.SRAM4(), offset, val); break;
    case BusDevice::SRAM5: /* [2048] */ byte_array_write_as_byte(m_rp2040.SRAM5(), offset, val); break;
    case BusDevice::APB: co_await m_rp2040.APB().write_byte(addr, val); break;
    case BusDevice::XIP: co_await m_rp2040.XIP().write_byte(addr, val); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().write_byte(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
  co_return;
}
Awaitable<void> AHB::write_halfword_internal(uint32_t addr, uint16_t val)
{
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  uint16_t out;
  switch(dev) {
    case BusDevice::ROM: /* [8192] */  break;
    case BusDevice::SRAM0: /* [32768] */ byte_array_write_as_halfword(m_rp2040.SRAM0(), offset, val); break;
    case BusDevice::SRAM1: /* [32768] */ byte_array_write_as_halfword(m_rp2040.SRAM1(), offset, val); break;
    case BusDevice::SRAM2: /* [32768] */ byte_array_write_as_halfword(m_rp2040.SRAM2(), offset, val); break;
    case BusDevice::SRAM3: /* [32768] */ byte_array_write_as_halfword(m_rp2040.SRAM3(), offset, val); break;
    case BusDevice::SRAM4: /* [2048] */ byte_array_write_as_halfword(m_rp2040.SRAM4(), offset, val); break;
    case BusDevice::SRAM5: /* [2048] */ byte_array_write_as_halfword(m_rp2040.SRAM5(), offset, val); break;
    case BusDevice::APB: co_await m_rp2040.APB().write_halfword(addr, val); break;
    case BusDevice::XIP: co_await m_rp2040.XIP().write_halfword(addr, val); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().write_halfword(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
}
Awaitable<void> AHB::write_word_internal(uint32_t addr, uint32_t val)
{
  // std::cout << "AHB::write_word_internal(" << std::hex << addr << std::dec << ")" << std::endl;
  auto devoffset = lookupBusDeviceAddress(addr);
  auto dev = std::get<0>(devoffset);
  auto offset = std::get<1>(devoffset);
  uint16_t out;
  // std::cout << "writing word to " << std::hex << addr << std::dec << " dev " << (int)dev << " offset " << offset << " val " << val << std::endl;
  switch(dev) {
    case BusDevice::ROM: /* [8192] */  break;
    case BusDevice::SRAM0: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM0(), offset, val); break;
    case BusDevice::SRAM1: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM1(), offset, val); break;
    case BusDevice::SRAM2: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM2(), offset, val); break;
    case BusDevice::SRAM3: /* [32768] */ byte_array_write_as_word(m_rp2040.SRAM3(), offset, val); break;
    case BusDevice::SRAM4: /* [2048] */ byte_array_write_as_word(m_rp2040.SRAM4(), offset, val); break;
    case BusDevice::SRAM5: /* [2048] */ byte_array_write_as_word(m_rp2040.SRAM5(), offset, val); break;
    case BusDevice::APB: co_await m_rp2040.APB().write_word(addr, val); break;
    case BusDevice::XIP: co_await m_rp2040.XIP().write_word(addr, val); break;
    case BusDevice::AHBLITE: m_rp2040.AHBLite().write_word(addr, val); break;
    default: throw ARMv6M::BusFault{addr};
  }
  // std::cout << "write_word(" << std::hex << addr << std::dec << ") completed" << std::endl;
}
