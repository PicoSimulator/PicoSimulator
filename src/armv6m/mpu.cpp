#include "armv6m/core.hpp"

using namespace ARMv6M;

// currently just a transparent interface to the next bus layer

ARMv6MCore::MPU::MPU(IAsyncReadWritePort<uint32_t> &bus, ARMv6MCore &core)
: m_bus_interface{bus}
, m_core{core}
, m_runner{bus_task().get_handle()}
{
}

#define READ(wordtype, ctype, out) {\
  ctype out2; \
  {if ((op.addr & 0xe000'0000) == 0xe000'0000) { \
    m_core.m_ppb.read_##wordtype (op.addr, out2); \
  } else { \
    out2 = co_await m_bus_interface.read_##wordtype (op.addr); \
  }}; out = out2;}

#define WRITE(wordtype) \
  if ((op.addr & 0xe000'0000) == 0xe000'0000) { \
    m_core.m_ppb.write_##wordtype (op.addr, op.data); \
  } else { \
    co_await m_bus_interface.write_##wordtype (op.addr, op.data); \
  }

Task ARMv6MCore::MPU::bus_task()
{
  while(true) {
    auto &op = co_await next_op();
    // std::cout << "MPU::bus_task() " << m_core.m_name << " " << op.addr << " " << op.optype << " " << op.data << std::endl;
    uint32_t out;
    switch(op.optype) {
      case MemoryOperation::READ_BYTE:
        READ(byte, uint8_t, out);
        op.return_value(out);
        break;
      case MemoryOperation::READ_HALFWORD:{
          READ(halfword, uint16_t, out);
          op.return_value(out);
        } break;
      case MemoryOperation::READ_WORD:{
          READ(word, uint32_t, out);
          op.return_value(out);
        } break;
      case MemoryOperation::WRITE_BYTE: {
        WRITE(byte);
        op.return_void();
      } break;
      case MemoryOperation::WRITE_HALFWORD: {
        WRITE(halfword);
        op.return_void();
      } break;
      case MemoryOperation::WRITE_WORD: {
        WRITE(word);
        op.return_void();
      } break;
    }
  }
}
