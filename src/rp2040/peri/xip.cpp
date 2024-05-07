#include "rp2040/peri/xip.hpp"
#include "armv6m/exception.hpp"

using namespace RP2040;

void XIP::tick() {
  // m_runner.m_handle.resume();
  if (m_stream_ctr && !m_stream_fifo.full()) {
    m_stream_fifo.push(m_flash[m_stream_addr]);
    m_stream_addr++;
    m_stream_ctr--;
  }

}

#define READ(wordtype, ctype, op)                                                                               \
{                                                                                                               \
  uint32_t set, line_no;                                                                                        \
  uint32_t line_offset;                                                                                         \
  switch((op.addr&0x0f00'0000)) {                                                                                 \
    case 0x0000'0000:                                                                                           \
      /*normal cache operation*/                                                                                \
      /*check for hit, update cache on miss*/                                                                   \
      m_acc_counter++;                                                                                          \
      if (cache_set_lookup(op.addr, set, line_no)) {                                                            \
        m_hit_counter++;                                                                                        \
        line_offset = set * 16 + line_no*8;                                                                     \
      } else {                                                                                                  \
        cache_set_choose_replacement(set, line_no);                                                             \
        line_offset = set * 16 + line_no*8;                                                                     \
        std::copy(&m_flash[(op.addr&0x00ff'fff8)] , &m_flash[(op.addr&0x00ff'fff8) + 8], &m_data[line_offset]); \
        m_cache_tags[set][line_no] = {true, cache_tag_decode(op.addr)};                                         \
      }                                                                                                         \
      op.return_value(*((ctype*)&m_data.data()[(line_offset + (op.addr&0x0000'0007))]));                        \
      break;                                                                                                    \
    case 0x0100'0000:                                                                                           \
      /* check for hit, don't update cache on miss */                                                           \
      m_acc_counter++;                                                                                          \
      if (cache_set_lookup(op.addr, set, line_no)) {                                                            \
        m_hit_counter++;                                                                                        \
        line_offset = set * 16 + line_no*8;                                                                     \
        op.return_value(*((ctype*)&m_data.data()[(line_offset + (op.addr&0x0000'0007))]));                      \
      } else {                                                                                                  \
        /* don't update cache on miss */                                                                        \
        op.return_value(*((ctype*)&m_flash.begin()[(op.addr&0x00ff'ffff)]));                                    \
      }                                                                                                         \
      break;                                                                                                    \
    case 0x0200'0000:                                                                                           \
      /* don't check for hit, always update cache */                                                            \
      m_acc_counter++;                                                                                          \
      set = cache_set_decode(op.addr);                                                                          \
      cache_set_choose_replacement(set, line_no);                                                               \
      line_offset = set * 16 + line_no*8;                                                                       \
      std::copy(&m_flash[(op.addr&0x00ff'fff8)] , &m_flash[(op.addr&0x00ff'fff8) + 8], &m_data[line_offset]);   \
      m_cache_tags[set][line_no] = {true, cache_tag_decode(op.addr)};                                           \
      [[fallthrough]];                                                                                          \
    case 0x0300'0000:                                                                                           \
      /* completely bypass cache */                                                                             \
      op.return_value(*((ctype*)&m_flash[(op.addr&0x00ff'ffff)]));                                              \
      break;                                                                                                    \
     case 0x0400'0000:                                                                                          \
     {                                                                                                          \
       switch(op.addr & 0x0000'3fff) {                                                                          \
         case 0x0000: /* CTRL */                                                                                \
           op.return_value(0x0000'000b);                                                                        \
           break;                                                                                               \
         case 0x0004:  /* FLUSH */                                                                              \
           op.return_value(0x0000'0000);                                                                        \
           break;                                                                                               \
         case 0x0008: /* STAT */                                                                                \
           op.return_value(0x0000'0003);                                                                        \
           break;                                                                                               \
         case 0x000c:                                                                                           \
           op.return_value(m_hit_counter.get());                                                                \
           break;                                                                                               \
         case 0x0010:                                                                                           \
           op.return_value(m_acc_counter.get());                                                                \
           break;                                                                                               \
         case 0x0014: /* STREAM_ADDR */                                                                         \
           op.return_value(m_stream_addr);                                                                      \
           break;                                                                                               \
         case 0x0018: /* STREAM_CTR */                                                                          \
           op.return_value(m_stream_ctr);                                                                       \
           break;                                                                                               \
         case 0x001c: /* STREAM_FIFO */                                                                         \
           op.return_value(m_stream_fifo.pop());                                                                \
           break;                                                                                               \
       }                                                                                                        \
     } break;                                                                                                   \
    case 0x0500'0000:                                                                                           \
      op.return_value(*((ctype*)&m_data[(op.addr&0x0000'3fff)]));                                               \
      break;                                                                                                    \
    case 0x0800'0000: /* XIP_SSI_BASE */                                                                        \
    {                                                                                                           \
      ctype out;                                                                                                \
      m_ssi.read_##wordtype(op.addr, out);                                                                      \
      op.return_value(out);                                                                                     \
      break;                                                                                                    \
    }                                                                                                           \
  }                                                                                                             \
}                                                                                                               \


Task XIP::bus_task(){
  while(true) {
    auto &op = co_await next_op();
    // std::cout << "got op" << std::endl;
    switch (op.optype) {
      case MemoryOperation::READ_BYTE:
        READ(byte, uint8_t, op);
        break;
      case MemoryOperation::READ_HALFWORD:
        READ(halfword, uint16_t, op);
        break;
      case MemoryOperation::READ_WORD:
        READ(word, uint32_t, op);
        break;
      case MemoryOperation::WRITE_BYTE:
        // m_flash[op.addr] = op.data;
        op.return_void();
        break;
      case MemoryOperation::WRITE_HALFWORD:
        // *(uint16_t*)&m_flash[op.addr] = op.data;
        op.return_void();
        break;
      case MemoryOperation::WRITE_WORD:
        // *(uint32_t*)&m_flash[op.addr] = op.data;
        if ((op.addr & 0x0f00'0000) ==  0x0800'0000) {
          m_ssi.write_word(op.addr, op.data);
        }
        op.return_void();
        break;
    }
  }
}

// Awaitable<void> XIP::write_byte(uint32_t addr, uint8_t in) 
// {
//   throw ARMv6M::UnimplementedFault{"XIP::write_byte"};
// }

// Awaitable<void> XIP::write_halfword(uint32_t addr, uint16_t in) 
// {
//   throw ARMv6M::UnimplementedFault{"XIP::write_halfword"};
// }

// Awaitable<void> XIP::write_word(uint32_t addr, uint32_t in) 
// {
//   uint32_t set, line_no;
//   switch(addr&0x0f00'0000) {
//     case 0x0000'0000:
//       if (cache_set_lookup(addr, set, line_no)) {
//         auto &[valid, line_tag] = m_cache_tags[set][line_no];
//         valid = 0;
//       }
//     case 0x0100'0000:
//     case 0x0200'0000:
//     case 0x0300'0000: co_return; // do nothing
//     case 0x0400'0000:
//     {
//       switch(addr & 0x0000'3fff) {
//         case 0x0000: // CTRL
//           // co_return 0x0000'000b;
//         case 0x0004: // FLUSH
//           flush();
//           // co_return 0x0000'0000;
//         case 0x0008: // STAT
//           // co_return 0x0000'0002;
//           co_return;
//         case 0x000c:
//           m_hit_counter.reset();
//           co_return ;
//         case 0x0010:
//           m_acc_counter.reset();
//           co_return ;
//         case 0x0014: // STREAM_ADDR
//           // 0x0000'0000;
//           co_return ;
//         case 0x0018: // STREAM_CTRL
//           // 0x0000'0000;
//           co_return ;
//       }
//     }
//     case 0x0500'0000:
//       m_data[addr&0x0000'3fff] = in;
//       co_return;
//     case 0x0800'0000: // XIP_SSI_BASE
//       m_ssi.write_word(addr, in);
//       co_return;
//   }
//   throw ARMv6M::UnimplementedFault{"XIP::write_word"};
// }

uint32_t XIP::cache_tag_decode(uint32_t addr) {
  return (addr >> 13) & 0x7ff;
}

uint32_t XIP::cache_set_decode(uint32_t addr) {
  return (addr >> 3) & 0x3ff;
}

bool XIP::cache_set_lookup(uint32_t addr, uint32_t &set, uint32_t &line_no) {
  uint32_t offset = addr & 0x07; // 3 bit byte address
  set = cache_set_decode(addr); // 10 bit set address
  uint32_t tag = cache_tag_decode(addr); // 11 bit tag
  auto &tag_set = m_cache_tags[set];
  for (int i = 0; i < 2; i++) {
    auto &[valid, line_tag] = tag_set[i];
    if (valid && (line_tag == tag)) {
      // hit
      line_no = i;
      return true;
    }
  }
  return false;
}

void XIP::cache_set_choose_replacement(uint32_t set, uint32_t &line_no) {
  // replacement policy to choose which line to replace
  // choose first invalid line
  auto &tag_set = m_cache_tags[set];
  for (int i = 0; i < 2; i++) {
    auto &[valid, line_tag] = tag_set[i];
    if (!valid) {
      line_no = i;
      return;
    }
  }
  // choose random line
  line_no = rand() & 1;
}

void XIP::flush() {
  for (auto &tag_set : m_cache_tags) {
    for (auto &tag : tag_set) {
      tag = {false, 0};
    }
  }
}