#include "armv6m/core.hpp"
#include "async.hpp"
#include "armv6m/exception.hpp"
#include "armv6m/opcodes.hpp"

#include <iostream>
#include <iomanip>
#include <bitset>
#include <cassert>

#define THUMB2 (1)

using namespace ARMv6M;

void ARMv6MCore::tick()
{
  // std::cout << "ARMv6MCore::tick() " << m_name << std::endl;
  if (m_waiting_for_tick) {
    m_waiting_for_tick = false;
    m_core_task.resume();
  }
  m_tickcnt++;
}

void ARMv6MCore::reset()
{
  m_in_reset = true;
}

void ARMv6MCore::dump() const
{
  for (int i = 0; i < 12; i++) {
    std::cout << "R" << std::setfill(' ') << std::setw(2) << std::left << i << "  " << std::hex << std::setw(8) << std::right << std::setfill(' ') << m_regs[i] << std::dec << std::endl;
  }
  std::cout << "SP   " << std::hex << std::setw(8) << std::right << std::setfill(' ') << SP() << std::dec << std::endl;
  std::cout << "MSP  " << std::hex << std::setw(8) << std::right << std::setfill(' ') << m_MSP << std::dec << std::endl;
  std::cout << "PSP  " << std::hex << std::setw(8) << std::right << std::setfill(' ') << m_PSP << std::dec << std::endl;
  std::cout << "LR   " << std::hex << std::setw(8) << std::right << std::setfill(' ') << LR() << std::dec << std::endl;
  std::cout << "PC   " << std::hex << std::setw(8) << std::right << std::setfill(' ') << PC() << std::dec << std::endl;

  std::cout << "APSR " << std::bitset<32>{m_APSR} << std::endl;
  std::cout << "     NZCV" << std::endl;
}

BusMaster ARMv6MCore::core_task()
{
  while(true){
    // VTOR = zeroes(32)
    for (int i = 0; i < 12; i++) {
      m_regs[i] = 0; // do we actually do this?
    }
    uint32_t vectortable = VTOR() = 0;
    m_threadMode = true;
    // LR() = UNKNOWN;
    m_regs[15] = 1;
    m_regs[14] = 0;
    m_regs[13] = 0;
    m_MSP = 0;
    m_PSP = 0;
    m_APSR = 0;
    m_IPSR = 0;
    m_EPSR = 0;
    m_PRIMASK = 0;
    m_CONTROL = 0;
    std::cout << "getting SP" << std::endl;
    m_MSP = co_await m_mpu_bus_interface.read_word(vectortable + 0x00); // SP
    SP() = m_MSP;
    std::cout << "getting PC" << std::endl;
    uint32_t start = co_await m_mpu_bus_interface.read_word(vectortable + 0x04); // RESET
    PC() = start;

    std::cout << std::hex << std::setw(8) << std::setfill('0') << SP() << std::dec << std::endl;
    std::cout << std::hex << std::setw(8) << std::setfill('0') << PC() << std::dec << std::endl;
    m_in_reset = false;
    while (!m_in_reset) {
      // std::cout << "starting instruction fetch number " << instruction_count++ << std::endl;
      uint8_t instr_incr = 2;
      try{
        // fetch
        uint32_t instr;
        if (!T())
        {
          // std::cout << "fetching arm" << std::endl;
          throw HardFault{"ARM mode not supported!"}; //ARMv6M does not support ARM mode
          instr = co_await m_mpu_bus_interface.read_word(instr_addr());
          instr_incr = 4;
        }
        else
        {
          // std::cout << "fetch thumb" << std::endl;
          instr = co_await m_mpu_bus_interface.read_halfword(instr_addr()) << 16;
          if ((instr & 0xf800'0000) >= 0xe100'0000) {
            // std::cout << "extended thumb instr" << std::endl;
            co_await next_tick();
            instr = instr | co_await m_mpu_bus_interface.read_halfword(instr_addr() + 2);
            instr_incr = 4;
          }
        }
        m_nextPC = PC() + instr_incr;
        // std::cout << "fetch complete " << m_name << " instr: "
        //   << std::hex << std::setw(8) << std::setfill('0') << instr << std::dec
        //   << " addr: "
        //   << std::hex << std::setw(8) << std::setfill('0') << PC() << std::dec
        //   << std::endl;

        // co_await exec_instr(instr);
        
        #define DO(x) {x}
        #define DO_DISAS(x) {std::cout << "D:" << std::setw(8) << std::hex << std::setfill('0') << PC() << std::dec << ": ";  {x}}
        #define DO_TRACE(x) {std::cout << "T:";  {x}}
        #define DO2(x, y) {x}{y}
        #define DONT(x) 

        #define OPCODE(prefix, fun) \
          case prefix: \
            fun(instr, DONT, DO, DONT, this); \
            break;

        #define REP0 OPCODE

        #define REP1(prefix, fun) \
          REP0(prefix##0, fun) \
          REP0(prefix##1, fun) \

        #define REP2(prefix, fun) \
          REP1(prefix##0, fun) \
          REP1(prefix##1, fun) \

        #define REP3(prefix, fun) \
          REP2(prefix##0, fun) \
          REP2(prefix##1, fun) \

        switch(instr >> 25) {
          ENUM_OPCODES(REP0)
        }
        #undef OPCODE
        #undef REP1
        #undef REP2
        #undef REP3
        #undef REP4
        PC() = m_nextPC;
        //check for PC special values?

        // decode
        // execute
        // writeback
        // if (m_name == "core-0" && m_tickcnt > 10460)
          // dump();
      } catch (ARMv6M::HardFault fault) {
        std::cout << "Hardfault on instruction fetch" << std::endl;
        dump();
        // std::flush();

        std::terminate();
        // std::exit(-1);
        // Exception Entry!
        R"____(
            // ExceptionEntry()
            // =================
            ExceptionEntry(integer ExceptionType)
            // NOTE: PushStack() can abandon memory accesses if a fault occurs during the stacking
            // sequence.
            // Exception entry is modified according to the behavior of a derived exception.
            PushStack(ExceptionType);
            ExceptionTaken(ExceptionType); // ExceptionType is encoded as its exception number
            // PushStack()
            // ===========
            PushStack(integer ExceptionType)
              if CONTROL.SPSEL == ‘1’ && CurrentMode == Mode_Thread then
                frameptralign = SP_process<2>;
                SP_process = (SP_process - 0x20) AND NOT(ZeroExtend(‘100’,32));
                frameptr = SP_process;
              else
                frameptralign = SP_main<2>;
                SP_main = (SP_main - 0x20) AND NOT(ZeroExtend(‘100’,32));
                frameptr = SP_main;
              /* only the stack locations, not the store order, are architected */
              MemA[frameptr,4] = R[0];
              MemA[frameptr+0x4,4] = R[1];
              MemA[frameptr+0x8,4] = R[2];
              MemA[frameptr+0xC,4] = R[3];
              MemA[frameptr+0x10,4] = R[12];
              MemA[frameptr+0x14,4] = LR;
              MemA[frameptr+0x18,4] = ReturnAddress(ExceptionType);
              MemA[frameptr+0x1C,4] = (XPSR<31:10>:frameptralign:XPSR<8:0>);
              if CurrentMode==Mode_Handler then
                LR = 0xFFFFFFF1<31:0>;
              else
                if CONTROL.SPSEL == ‘0’ then
                  LR = 0xFFFFFFF9<31:0>;
                else
                  LR = 0xFFFFFFFD<31:0>;
              return;

            // ExceptionTaken()
            // ================
            ExceptionTaken(integer ExceptionNumber)
              for n = 0 to 3
              R[n] = bits(32) UNKNOWN; // Original values pushed on stack
              R[12] = bits(32) UNKNOWN;
              APSR = bits(32) UNKNOWN;
              CurrentMode = Mode_Handler; // Enter Handler Mode, now Privileged
              IPSR<5:0> = ExceptionNumber<5:0>; // Update IPSR to this exception
              CONTROL.SPSEL = ‘0’; // Current stack is now SP main
              // CONTROL.nPRIV unchanged
              ExceptionActive[ExceptionNumber] = ‘1’; // Set exception as being active
              SCS_UpdateStatusRegs(); // Update SCS registers
              SetEventRegister(); // See WFE instruction for details
              InstructionSynchronizationBarrier(‘1111’);
              bits(32) vectortable = VTOR;
              start = MemA[vectortable+4*ExceptionNumber,4]; // Load handler address
              BLXWritePC(start); // Start execution of handler

              // ReturnAddress()
              // ===============
              bits(32) ReturnAddress(integer ExceptionType)
              // Returns the following values based on the exception cause
              // NOTE: ReturnAddress() is always halfword aligned, meaning bit<0> is always zero
                if ExceptionType == NMI then result = NextInstrAddr();
                elsif ExceptionType == HardFault then
                  result = if IsExceptionSynchronous() then ThisInstrAddr() else NextInstrAddr();
                elsif ExceptionType == SVCall then result = NextInstrAddr();
                elsif ExceptionType == PendSV then result = NextInstrAddr();
                elsif ExceptionType == SysTick then result = NextInstrAddr();
                elsif ExceptionType >= 16 then // External interrupt
                  result = NextInstrAddr();
                else
                  assert(FALSE); // Unknown exception number
                return result;
        )____";
        uint32_t sp, frameptr;
        if (m_CONTROL & CONTROL::SPSEL && m_threadMode) {
          sp = m_PSP;
          m_PSP -= 0x20;
          m_PSP &= ~0x100;
          frameptr = m_PSP;
        } else {
          sp = m_MSP;
          m_MSP -= 0x20;
          m_MSP &= ~0x100;
          frameptr = m_MSP;
        }
        m_bus_interface.write_word(frameptr, m_regs[0]);
        m_bus_interface.write_word(frameptr + 0x4, m_regs[1]);
        m_bus_interface.write_word(frameptr + 0x8, m_regs[2]);
        m_bus_interface.write_word(frameptr + 0xC, m_regs[3]);
        m_bus_interface.write_word(frameptr + 0x10, m_regs[12]);
        m_bus_interface.write_word(frameptr + 0x14, LR());
        m_bus_interface.write_word(frameptr + 0x18, ReturnAddress(ExceptionType::HardFault));
        m_bus_interface.write_word(frameptr + 0x1C, (XPSR() & 0xFFFFFC00) | ((sp & 0x2) << 8) | (XPSR() & 0x1FF));
        if (m_threadMode) {
          LR() = 0xFFFFFFF1;
        } else {
          if (!(m_CONTROL & CONTROL::SPSEL)) {
            LR() = 0xFFFFFFF9;
          } else {
            LR() = 0xFFFFFFFD;
          }
        }

      }
      co_await next_tick();
    }
  }
}

Awaitable<void> ARMv6MCore::exec_instr(uint32_t instr)
{
  #define DO(x) {x}
  #define DO_DISAS(x) {std::cout << "D:" << std::setw(8) << std::hex << std::setfill('0') << PC() << std::dec << ": ";  {x}}
  #define DO_TRACE(x) {std::cout << "T:";  {x}}
  #define DO2(x, y) {x}{y}
  #define DONT(x) 

  #define OPCODE(prefix, fun) \
    case prefix: \
      fun(instr, DONT, DO, DONT, this); \
      break;

  #define REP0 OPCODE

  #define REP1(prefix, fun) \
    REP0(prefix##0, fun) \
    REP0(prefix##1, fun) \

  #define REP2(prefix, fun) \
    REP1(prefix##0, fun) \
    REP1(prefix##1, fun) \

  #define REP3(prefix, fun) \
    REP2(prefix##0, fun) \
    REP2(prefix##1, fun) \

  switch(instr >> 25) {
    ENUM_OPCODES(REP0)
  }
  #undef OPCODE
  #undef REP1
  #undef REP2
  #undef REP3
  #undef REP4
  co_return;
}


PortState ARMv6MCore::PPB::read_byte(uint32_t addr, uint8_t &out){ throw ARMv6M::BusFault(); }
PortState ARMv6MCore::PPB::read_halfword(uint32_t addr, uint16_t &out){ throw ARMv6M::BusFault(); }
PortState ARMv6MCore::PPB::read_word(uint32_t addr, uint32_t &out){
  std::cout << "PPB::read_word(" << std::hex << addr << std::dec << ")" << std::endl;
  switch(addr) {
    case 0xE000ED00: // CPUID
      out = 0x410CC601;
      break;
    case 0xE000ED04: // ICSR
      out = 0;
      break;
    case 0xE000ED08: // VTOR
      out = m_core.VTOR();
      break;
    case 0xE000ED0C: // AIRCR
      out = 0;
      break;
    case 0xE000ED10: // SCR
      out = 0;
      break;
    case 0xE000ED14: // CCR
      out = 0;
      break;
    case 0xE000ED18: // SHPR1
      out = 0;
      break;
    case 0xE000ED1C: // SHPR2
      out = 0;
      break;
    case 0xE000ED20: // SHPR3
      out = 0;
      break;
    case 0xE000ED24: // SHCSR
      out = 0;
      break;
    case 0xE000ED28: // CFSR
      out = 0;
      break;
    case 0xE000ED2C: // HFSR
      out = 0;
      break;
    case 0xE000ED30: // DFSR
      out = 0;
      break;
    case 0xE000ED34: // MMFAR
      out = 0;
      break;
    case 0xE000ED38: // BFAR
      out = 0;
      break;
    case 0xE000ED3C: // AFSR
      out = 0;
      break;
    case 0xE000ED40: // PFR0
      out = 0;
      break;
    case 0xE000ED44: // PFR1
      out = 0;
      break;
    case 0xE000ED48: // DFR0
      out = 0;
      break;
    case 0xE000ED4C: // AFR0
      out = 0;
      break;
    case 0xE000ED50: // MMFR0
      out = 0;
      break;
    case 0xE000ED54: // MMFR1
      out = 0;
      break;
  }
  return PortState::SUCCESS;
}
PortState ARMv6MCore::PPB::write_byte(uint32_t addr, uint8_t in){ throw ARMv6M::BusFault(); }
PortState ARMv6MCore::PPB::write_halfword(uint32_t addr, uint16_t in){ throw ARMv6M::BusFault(); }
PortState ARMv6MCore::PPB::write_word(uint32_t addr, uint32_t in){
  switch(addr) {
    case 0xE000ED00: // CPUID
      break;
    case 0xE000ED04: // ICSR
      break;
    case 0xE000ED08: // VTOR
      m_core.VTOR() = in;
      break;
    case 0xE000ED0C: // AIRCR
      break;
    case 0xE000ED10: // SCR
      break;
    case 0xE000ED14: // CCR
      break;
    case 0xE000ED18: // SHPR1
      break;
    case 0xE000ED1C: // SHPR2
      break;
    case 0xE000ED20: // SHPR3
      break;
    case 0xE000ED24: // SHCSR
      break;
    case 0xE000ED28: // CFSR
      break;
    case 0xE000ED2C: // HFSR
      break;
    case 0xE000ED30: // DFSR
      break;
    case 0xE000ED34: // MMFAR
      break;
    case 0xE000ED38: // BFAR
      break;
    case 0xE000ED3C: // AFSR
      break;
    case 0xE000ED40: // PFR0
      break;
    case 0xE000ED44: // PFR1
      break;
    case 0xE000ED48: // DFR0
      break;
    case 0xE000ED4C: // AFR0
      break;
    case 0xE000ED50: // MMFR0
      break;
    case 0xE000ED54: // MMFR1
      break;
  }
  return PortState::SUCCESS;
}