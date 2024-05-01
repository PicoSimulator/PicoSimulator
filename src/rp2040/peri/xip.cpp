#include "rp2040/peri/xip.hpp"
#include "armv6m/exception.hpp"

using namespace RP2040;

void XIP::tick() {
  if (m_stream_ctr && !m_stream_fifo.full()) {
    m_stream_fifo.push(m_flash[m_stream_addr]);
    m_stream_addr++;
    m_stream_ctr--;
  }

}

Awaitable<uint8_t> XIP::read_byte(uint32_t addr) 
{
  switch(addr&0x0f00'0000) {
    case 0x0000'0000:
    case 0x0100'0000:
    case 0x0200'0000:
    case 0x0300'0000:
      co_return ((uint8_t*)m_flash.begin())[(addr&0x00ff'ffff)];
  }
}

Awaitable<uint16_t> XIP::read_halfword(uint32_t addr) 
{
  switch(addr&0x0f00'0000) {
    case 0x0000'0000:
    case 0x0100'0000:
    case 0x0200'0000:
    case 0x0300'0000:
      co_return ((uint16_t*)m_flash.begin())[(addr&0x00ff'ffff)/2];
  }
}

Awaitable<uint32_t> XIP::read_word(uint32_t addr) 
{
  switch(addr&0x0f00'0000) {
    case 0x0000'0000:
    case 0x0100'0000:
    case 0x0200'0000:
    case 0x0300'0000:
      co_return ((uint32_t*)m_flash.begin())[(addr&0x00ff'ffff)/4];
    case 0x0400'0000:
    {
      switch(addr & 0x0000'3fff) {
        case 0x0000: // CTRL
          co_return 0x0000'000b;
        case 0x0004: // FLUSH
          co_return 0x0000'0000;
        case 0x0008: // STAT
          co_return 0x0000'0002;
        case 0x000c:
          co_return m_hit_counter.get();
        case 0x0010:
          co_return m_acc_counter.get();
        case 0x0014: // STREAM_ADDR
          co_return m_stream_addr;
        case 0x0018: // STREAM_CTR
          co_return m_stream_ctr;
        case 0x001c: // STREAM_FIFO
          co_return m_stream_fifo.pop();
      }
    }
    case 0x0500'0000:
      co_return ((uint32_t*)m_data.data())[(addr&0x0000'3fff) >> 2];
    case 0x0800'0000: // XIP_SSI_BASE
    {
      uint32_t out;
      m_ssi.read_word(addr, out);
      co_return out;
    }
  }
  throw ARMv6M::UnimplementedFault{"XIP::read_word"};
}

Awaitable<void> XIP::write_byte(uint32_t addr, uint8_t in) 
{
  throw ARMv6M::UnimplementedFault{"XIP::write_byte"};
}

Awaitable<void> XIP::write_halfword(uint32_t addr, uint16_t in) 
{
  throw ARMv6M::UnimplementedFault{"XIP::write_halfword"};
}

Awaitable<void> XIP::write_word(uint32_t addr, uint32_t in) 
{
  switch(addr&0x0f00'0000) {
    case 0x0000'0000:
    case 0x0100'0000:
    case 0x0200'0000:
    case 0x0300'0000: co_return; // do nothing
    case 0x0400'0000:
    {
      switch(addr & 0x0000'3fff) {
        case 0x0000: // CTRL
          // co_return 0x0000'000b;
        case 0x0004: // FLUSH
          // co_return 0x0000'0000;
        case 0x0008: // STAT
          // co_return 0x0000'0002;
          co_return;
        case 0x000c:
          m_hit_counter.reset();
          co_return ;
        case 0x0010:
          m_acc_counter.reset();
          co_return ;
        case 0x0014: // STREAM_ADDR
          // 0x0000'0000;
          co_return ;
        case 0x0018: // STREAM_CTRL
          // 0x0000'0000;
          co_return ;
      }
    }
    case 0x0500'0000:
      m_data[addr&0x0000'3fff] = in;
      co_return;
    case 0x0800'0000: // XIP_SSI_BASE
      m_ssi.write_word(addr, in);
      co_return;
  }
  throw ARMv6M::UnimplementedFault{"XIP::write_word"};
}