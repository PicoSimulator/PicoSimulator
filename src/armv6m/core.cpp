#include "armv6m/core.hpp"
#include "async.hpp"
#include "armv6m/exception.hpp"

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
    uint32_t vectortable = 0;
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
    m_MSP = co_await m_mpu_bus_interface.read_word(vectortable + 0x00); // SP
    SP() = m_MSP;
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
        std::cout << "fetch complete " << m_name << " instr: "
          << std::hex << std::setw(8) << std::setfill('0') << instr << std::dec
          << " addr: "
          << std::hex << std::setw(8) << std::setfill('0') << PC() << std::dec
          << std::endl;

        co_await exec_instr(instr);
        PC() = m_nextPC;
        //check for PC special values?

        // decode
        // execute
        // writeback
        // if (m_name == "core-0" && m_tickcnt > 10460)
          dump();
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

#define OPCODE_UNDEFINED(opcode) \
  { \
    std::cout << "Undefined opcode " << std::hex << opcode << std::endl; \
    throw HardFault{"Undefined OpCode"}; \
  }

#define SETFLAGS_NZ(result) \
  ({m_APSR &= ~(APSR::NEGATIVE | APSR::ZERO); \
  if (result & 0x80000000) { \
    m_APSR |= APSR::NEGATIVE; \
  } \
  if (result == 0) { \
    m_APSR |= APSR::ZERO; \
  }})

#define SETFLAGS_NZC(result, carry) \
  ({m_APSR &= ~(APSR::NEGATIVE | APSR::ZERO | APSR::CARRY); \
  if (result & 0x80000000) { \
    m_APSR |= APSR::NEGATIVE; \
  } \
  if (result == 0) { \
    m_APSR |= APSR::ZERO; \
  } \
  if (carry) { \
    m_APSR |= APSR::CARRY; \
  }})

#define SETFLAGS_NZCV(result, carry, overflow) \
  ({m_APSR &= ~(APSR::NEGATIVE | APSR::ZERO | APSR::CARRY | APSR::OVERFLOW); \
  if (result & 0x80000000) { \
    m_APSR |= APSR::NEGATIVE; \
  } \
  if (result == 0) { \
    m_APSR |= APSR::ZERO; \
  } \
  if (carry) { \
    m_APSR |= APSR::CARRY; \
  } \
  if (overflow) { \
    m_APSR |= APSR::OVERFLOW; \
  }})

#define OPCODE_0000_0xx_lsl(opcode) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x07;\
    uint32_t Rm = (opcode >> 19) & 0x07;\
    uint32_t imm5 = (opcode >> 22) & 0x1f;\
    uint32_t Rm_val = get_reg(Rm);\
    bool carry_out = Rm_val & (1<<(32-imm5));\
    if (imm5 != 0) {\
      Rm_val <<= imm5;\
    }\
    std::cout << "LSL R" << Rd << ", R" << Rm << ", #" << imm5 << " ; R" << Rd << " = " << Rm_val << std::endl;\
    set_reg(Rd, Rm_val);\
    SETFLAGS_NZC(Rm_val, carry_out);\
  }

#define OPCODE_0000_1xx_lsr(opcode) \
{\
    uint32_t Rd = (opcode >> 16) & 0x07;\
    uint32_t Rm = (opcode >> 19) & 0x07;\
    uint32_t imm5 = (opcode >> 22) & 0x1f;\
    uint32_t Rm_val = get_reg(Rm);\
    bool carry_out = Rm_val & (1<<(imm5));\
    if (imm5 != 0) {\
      Rm_val >>= imm5;\
    }\
    std::cout << "LSR R" << Rd << ", R" << Rm << ", #" << imm5 << " ; R" << Rd << " = " << Rm_val << std::endl;\
    set_reg(Rd, Rm_val);\
    SETFLAGS_NZC(Rm_val, carry_out);\
}

#define OPCODE_0001_100_add(opcode) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t Rm = (opcode >> 22) & 0x7;\
    uint32_t Rn_val = get_reg(Rn);\
    uint32_t Rm_val = get_reg(Rm);\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, Rm_val, 0);\
    std::cout << "ADD R" << Rd << ", R" << Rn << ", R" << Rm << " ; R" << Rd << " = " << result << std::endl;\
    set_reg(Rd, result);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }

#define OPCODE_0001_110_add(opcode) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t imm3 = (opcode >> 22) & 0x7;\
    uint32_t Rn_val = get_reg(Rn);\
    std::cout << "ADD R" << Rd << ", R" << Rn << ", #" << imm3 << std::endl;\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, imm3, 0);\
    set_reg(Rd, result);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }

#define OPCODE_0001_111_sub(opcode) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t imm3 = (opcode >> 22) & 0x7;\
    uint32_t Rn_val = get_reg(Rn);\
    std::cout << "SUB R" << Rd << ", R" << Rn << ", #" << imm3 << std::endl;\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, ~imm3, 1);\
    set_reg(Rd, result);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }



#define OPCODE_0010_0xx_mov(opcode) \
  { \
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t Rd = (opcode >> 24) & 0x07;\
    std::cout << "MOV R" << Rd << ", #" << imm8 << " ; R" << Rd << " = " << imm8 << std::endl;\
    set_reg(Rd, imm8);\
    if (imm8) {\
      m_APSR &= ~APSR::ZERO;\
    } else {\
      m_APSR |= APSR::ZERO;\
    }\
  }

std::tuple<uint32_t, bool, bool> AddWithCarry(uint32_t a, uint32_t b, bool carry_in)
{
  uint64_t result = (uint64_t)a + (uint64_t)b + (uint64_t)carry_in;
  bool carry_out = result & 0x100000000;
  bool overflow = ((a & 0x80000000) == (b & 0x80000000)) && ((a & 0x80000000) != (result & 0x80000000));
  return {result, carry_out, overflow};
}

#define OPCODE_0010_1xx_cmp(opcode) \
  { \
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t simm32 = ~imm8;\
    uint32_t Rn = (opcode >> 24) & 0x07;\
    uint32_t Rn_val = get_reg(Rn);\
    std::cout << "CMP R" << Rn << ", #" << imm8 << std::endl;\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, simm32, 1);\
    std::cout << carry << std::endl;\
    m_APSR &= ~(APSR::NEGATIVE | APSR::ZERO | APSR::CARRY | APSR::OVERFLOW);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }

#define OPCODE_0011_0xx_add(opcode) \
  { \
    uint32_t Rdn = (opcode >> 24) & 0x7;\
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t Rn_val = get_reg(Rdn);\
    std::cout << "ADD R" << Rdn << ", R" << Rdn << ", #" << imm8 << std::endl;\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, imm8, 0);\
    set_reg(Rdn, result);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }

#define OPCODE_0011_1xx_sub(opcode) \
  { \
    uint32_t Rdn = (opcode >> 24) & 0x7;\
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t Rn_val = get_reg(Rdn);\
    auto [result, carry, overflow] = AddWithCarry(Rn_val, ~imm8, 1);\
    std::cout << "SUB R" << Rdn << ", R" << Rdn << ", #" << imm8 << " ; R" << Rdn << " = " << result << std::endl;\
    set_reg(Rdn, result);\
    SETFLAGS_NZCV(result, carry, overflow);\
  }

#define OPCODE_0100_00x_arith(opcode) \
{\
  uint8_t sub_opcode = (opcode >> 22) & 0xf;\
  switch(sub_opcode) {\
    case 0b0000:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      uint32_t Rdn_val = get_reg(Rdn);\
      uint32_t Rm_val = get_reg(Rm);\
      Rdn_val &= Rm_val; \
      std::cout << "AND R"<< Rdn << ", R" << Rm << " ; R" << Rm << " = " << Rdn_val << std::endl;\
      set_reg(Rdn, Rdn_val); \
      SETFLAGS_NZ(Rdn_val);\
    } break; \
    case 0b0001:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      uint32_t Rdn_val = get_reg(Rdn);\
      uint32_t Rm_val = get_reg(Rm);\
      Rdn_val ^= Rm_val; \
      std::cout << "EOR R"<< Rdn << ", R" << Rm << " ; R" << Rm << " = " << Rdn_val << std::endl;\
      set_reg(Rdn, Rdn_val); \
      SETFLAGS_NZ(Rdn_val);\
    } break; \
    case 0b1000:{\
      uint32_t Rn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      uint32_t Rn_val = get_reg(Rn);\
      uint32_t Rm_val = get_reg(Rm);\
      uint32_t result = Rn_val & Rm_val;\
      std::cout << "TST R"<< Rn << ", R" << Rm << std::endl;\
      SETFLAGS_NZ(result);\
    } break;\
    case 0b1001:{\
      uint32_t Rd = (opcode >> 16) & 0x07;\
      uint32_t Rn = (opcode >> 19) & 0x07;\
      uint32_t Rn_val = get_reg(Rn);\
      auto [result, carry, overflow] = AddWithCarry(~Rn_val, 0, 1);\
      std::cout << "RSB R"<< Rd << ", R" << Rn << std::endl;\
      set_reg(Rd, result); \
      SETFLAGS_NZCV(result, carry, overflow);\
    } break;\
    case 0b1010:{\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      uint32_t Rn = (opcode >> 16) & 0x07;\
      uint32_t Rm_val = get_reg(Rm);\
      uint32_t Rn_val = get_reg(Rn);\
      auto [result, carry, overflow] = AddWithCarry(Rn_val, ~Rm_val, true);\
      std::cout << "CMP R"<< Rm << ", R" << Rn << std::endl;\
      SETFLAGS_NZCV(result, carry, overflow);\
    } break;\
    case 0b1100:{\
      uint32_t Rdn = (opcode >> 16) & 0x7;\
      uint32_t Rm = (opcode >> 19) & 0x7;\
      uint32_t Rdn_val = get_reg(Rdn);\
      uint32_t Rm_val = get_reg(Rm);\
      Rdn_val |= Rm_val;\
      std::cout << "ORR R" << Rdn << ", R" << Rm << std::endl;\
      set_reg(Rdn, Rdn_val);\
      SETFLAGS_NZ(Rdn_val);\
    } break;\
    case 0b1110:{\
      uint32_t Rd = (opcode >> 16) & 0x7;\
      uint32_t Rn = (opcode >> 19) & 0x7;\
      uint32_t Rd_val = get_reg(Rd);\
      uint32_t Rn_val = get_reg(Rn);\
      Rd_val &= ~Rn_val;\
      std::cout << "BIC R" << Rd << ", R" << Rn << std::endl;\
      set_reg(Rd, Rd_val);\
      SETFLAGS_NZ(Rd_val);\
    } break;\
    case 0b1111:{\
      uint32_t Rd = (opcode >> 16) & 0x7;\
      uint32_t Rn = (opcode >> 19) & 0x7;\
      std::cout << "MVN R" << Rd << ", R" << Rn << std::endl;\
      set_reg(Rd, ~get_reg(Rn));\
      SETFLAGS_NZC(~get_reg(Rn), false);\
    } break;\
    default: throw HardFault{"Unknown arith instr"};\
  }\
}

#define OPCODE_0100_011_mov(opcode) \
  { \
    if ((opcode & 1<<24) == 0){\
      uint32_t Rd = ((opcode >> 16) & 0x7) | ((opcode >> 20) & 0x08) ; \
      uint32_t Rm = (opcode >> 19) & 0xf; \
      uint32_t RegVal = get_reg(Rm);\
      std::cout << "MOV R" << Rd << ", R" << Rm << " ; R" << Rd << " = " << RegVal << std::endl;\
      if (Rd == 15) {\
        m_nextPC = RegVal;\
      } else {\
        set_reg(Rd, RegVal); \
        SETFLAGS_NZ(RegVal);\
      }\
    } else if ((opcode & 1<<23) == 0) {\
      uint8_t Rm = (opcode>>19) & 0xf;\
      m_nextPC = get_reg(Rm);\
    }\
  }

#define OPCODE_0100_1xx_load(opcode) \
  { \
    uint32_t imm8 = (opcode >> 16) & 0xFF; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    uint32_t addr = (PC() & ~3) + imm8*4 + 4; \
    std::cout << "LDR R" << Rt << ", [PC, #" << imm8*4 << "] (" << std::hex << addr << std::dec <<")" << std::endl;\
    uint32_t data = co_await m_mpu_bus_interface.read_word(addr); \
    set_reg(Rt, data); \
  }

#define OPCODE_0101_000_store(opcode) \
  { \
    uint32_t imm5 = (instr >> 22) & 0x1F; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rm = instr & 0x7; \
    uint32_t addr = get_reg(Rn) + imm5; \
    uint32_t data = get_reg(Rm); \
    co_await m_mpu_bus_interface.write_word(addr, data); \
  }

#define OPCODE_0110_0xx_store(opcode) \
  { \
    uint32_t imm5 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm5*4; \
    uint32_t data = get_reg(Rt); \
    std::cout << "STR R" << Rt << ", [R" << Rn << ", #" << imm5*4 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    std::cout << &m_mpu_bus_interface << std::endl;\
    co_await m_mpu_bus_interface.write_word(addr, data); \
  }

#define OPCODE_0110_1xx_load(opcode) \
  { \
    uint32_t imm5 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm5*4; \
    std::cout << "LDR R" << Rt << ", [R" << Rn << ", #" << imm5*4 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    uint32_t data = co_await m_mpu_bus_interface.read_word(addr); \
    set_reg(Rt, data); \
  }

#define OPCODE_0111_0xx_strb(opcode) \
  { \
    uint32_t imm5 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm5; \
    uint32_t data = get_reg(Rt); \
    std::cout << "STRB R" << Rt << ", [R" << Rn << ", #" << imm5 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    co_await m_mpu_bus_interface.write_byte(addr, data); \
  }

#define OPCODE_0111_1xx_ldrb(opcode) \
  { \
    uint32_t imm32 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm32; \
    uint32_t data = co_await m_mpu_bus_interface.read_byte(addr);\
    std::cout << "LDRB R" << Rt << ", [R" << Rn << ", #" << imm32 << "] (" << std::hex << addr << "," << data << std::dec << ")" << std::endl;\
    set_reg(Rt, data); \
  }

#define OPCODE_1000_0xx_strh(opcode) \
  { \
    uint32_t imm32 = (opcode >> 21) & 0x3E; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm32; \
    uint32_t data = get_reg(Rt); \
    std::cout << "STRH R" << Rt << ", [R" << Rn << ", #" << imm32 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    co_await m_mpu_bus_interface.write_halfword(addr, data); \
  }

#define OPCODE_1000_1xx_ldrh(opcode) \
  { \
    uint32_t imm32 = (opcode >> 21) & 0x3E; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    uint32_t addr = get_reg(Rn) + imm32; \
    uint32_t data = co_await m_mpu_bus_interface.read_halfword(addr);\
    std::cout << "LDRH R" << Rt << ", [R" << Rn << ", #" << imm32 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    set_reg(Rt, data); \
  }

#define OPCODE_1001_0xx_str(opcode) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    uint32_t Rn = 13;\
    uint32_t addr = get_reg(Rn) + imm32; \
    std::cout << "STR R" << Rt << ", [SP, #" << imm32 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    co_await m_mpu_bus_interface.write_word(addr, get_reg(Rt)); \
  }

#define OPCODE_1001_1xx_ldr(opcode) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    uint32_t Rn = 13;\
    uint32_t addr = get_reg(Rn) + imm32; \
    std::cout << "LDR R" << Rt << ", [SP, #" << imm32 << "] (" << std::hex << addr << std::dec << ")" << std::endl;\
    uint32_t data = co_await m_mpu_bus_interface.read_word(addr); \
    set_reg(Rt, data); \
  }

#define OPCODE_1010_0xx_add(opcode) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rd = (opcode >> 24) & 0x07; \
    uint32_t addr = PC() & ~0x03 + imm32; \
    std::cout << "ADD R" << Rd << ", PC, #" << imm32 << "( " << std::hex << addr << std::dec << " )" << std::endl;\
    set_reg(Rd, addr);\
  }

#define OPCODE_1010_1xx_spadd(opcode) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rd = (opcode >> 24) & 0x7;\
    auto [result, carry, overflow] = AddWithCarry(SP(), imm32, 0);\
    std::cout << "ADD R" << Rd << ", SP, #" << imm32 << "( " << std::hex << result << std::dec << " )" << std::endl;\
    set_reg(Rd, result);\
  }

#define OPCODE_1011_000_sp(opcode) \
  { \
    switch(opcode & 0x0080'0000){\
      case 0x0000'0000: {\
        uint32_t imm32 = (opcode >> 14) & 0x1fc;\
        auto [newsp, carry, overflow] = AddWithCarry(SP(), imm32, 0);\
        std::cout << "ADD SP, #" << imm32 << std::endl;\
        SP() = newsp;\
      } break;\
      case 0x0080'0000: {\
        uint32_t imm32 = (opcode >> 14) & 0x1fc;\
        auto [newsp, carry, overflow] = AddWithCarry(SP(), ~imm32, 1);\
        std::cout << "SUB SP, #" << imm32 << std::endl;\
        SP() = newsp;\
      } break;\
      default: throw HardFault{"SPSE something or other"};\
    }\
  }

#define OPCODE_1011_001_ext(opcode) \
  { \
    uint32_t Rd = (opcode >> 16) & 0xf;\
    uint32_t Rm = (opcode >> 19) & 0xf;\
    uint32_t Rm_val = get_reg(Rm);\
    switch(opcode & 0x00c0'0000){\
      case 0x0000'0000: {\
        std::cout << "SXTH R" << Rd << ", R" << Rm << std::endl;\
        Rm_val = Rm_val&0xffff;\
        Rm_val |= 0 - (Rm_val&0x8000);\
        set_reg(Rd, Rm_val);\
      } break;\
      case 0x0040'0000: {\
        std::cout << "SXTB R" << Rd << ", R" << Rm << std::endl;\
        Rm_val = Rm_val&0xff;\
        Rm_val |= 0 - (Rm_val&0x80);\
        set_reg(Rd, Rm_val);\
      } break;\
      case 0x0080'0000: {\
        std::cout << "UXTH R" << Rd << ", R" << Rm << std::endl;\
        Rm_val = Rm_val&0xffff;\
        set_reg(Rd, Rm_val);\
      } break;\
      case 0x00c0'0000: {\
        std::cout << "UXTB R" << Rd << ", R" << Rm << std::endl;\
        Rm_val = Rm_val&0xff;\
        set_reg(Rd, Rm_val);\
      } break;\
      default: throw HardFault{"Sign Extenison something or other"};\
    }\
  }

#define OPCODE_1011_010_pushm(opcode) \
  {\
    uint32_t regs_list = (opcode >> 16) & 0xff | (opcode >> 10) & 0x4000;\
    uint32_t addr = SP();\
    std::cout << "PUSH {" << std::bitset<16>{regs_list} << "}" << std::endl;\
    for (int i = 14; i >= 0; i--) {\
      if ((regs_list & (1<<i)) == 0) continue; \
      co_await m_mpu_bus_interface.write_word(addr-4, get_reg(i));\
      addr -= 4;\
    }\
    SP() = addr;\
  }

#define OPCODE_1011_101_rev(opcode) \
  {\
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rm = (opcode >> 19) & 0x7;\
    uint32_t Rm_val = get_reg(Rm);\
    switch(opcode & 0x01c0'0000){\
      case 0x0000'0000:\
        Rm_val = ((Rm_val & 0xff) << 24) | ((Rm_val & 0xff00) << 8) | ((Rm_val & 0xff0000) >> 8) | ((Rm_val & 0xff000000) >> 24);\
        std::cout << "REV "; break; \
      case 0x0040'0000:\
        Rm_val = ((Rm_val & 0xffff) << 16) | ((Rm_val & 0xffff0000) >> 8);\
        std::cout << "REV16 "; break; \
      case 0x00c0'0000:\
        Rm_val = ((Rm_val & 0xffff) << 16) | ((Rm_val & 0xffff0000) >> 8);\
        std::cout << "REVSH "; \
      default: \
        throw HardFault{"bad REV instr"}; \
    }\
    std::cout << "R" << Rd << ", R" << Rm << std::endl;\
    set_reg(Rd, Rm_val);\
  }

#define OPCODE_1011_110_popm(opcode) \
  {\
    uint32_t regs_list = (opcode >> 16) & 0xff | (opcode >> 9) & 0x8000;\
    uint32_t addr = SP();\
    std::cout << "POP {" << std::bitset<16>{regs_list} << "}" << std::endl;\
    for (int i = 0; i <= 7; i++) {\
      if ((regs_list & (1<<i)) == 0) continue; \
      std::cout << "popping from " << std::hex << addr << std::dec << std::endl;\
      set_reg(i, co_await m_mpu_bus_interface.read_word(addr));\
      addr += 4;\
    }\
    if (regs_list & (1<<15)) {\
      std::cout << "popping from " << std::hex << addr << std::dec << std::endl;\
      m_nextPC = co_await m_mpu_bus_interface.read_word(addr);\
      std::cout << "new PC " << std::hex << m_nextPC << std::dec << std::endl;\
      addr += 4;\
    }\
    std::cout << "POP {" << std::bitset<16>{regs_list} << "}" << std::endl;\
    SP() = addr;\
  }

#define OPCODE_1011_111_misc(opcode) \
  { \
    if (opcode & (1<<24)){\
      /*being lazy not checking for opb==0*/\
      uint32_t opa = (opcode >> 20) & 0xf;\
      switch(opa) {\
        case 0b0000: /*NOP*/ \
          std::cout << "NOP" << std::endl;\
          break;\
        case 0b0001: /*YIELD*/ \
          std::cout << "YIELD" << std::endl;\
          break;\
        case 0b0010: /*WFE*/ \
          std::cout << "WFE" << std::endl;\
          break;\
        case 0b0011: /*WFI*/ \
          std::cout << "WFI" << std::endl;\
          break;\
        case 0b0100: /*SEV*/ \
          std::cout << "SEV" << std::endl;\
          break;\
        default: throw HardFault{"Undefined opcode"};\
      }\
    } else {\
      /*BKPT*/\
      uint32_t imm8 = (opcode >> 16) & 0xFF; \
      std::cout << "BKPT #" << imm8 << std::endl;\
      throw HardFault{"BKPT not implemented"};\
    }\
  }

#define OPCODE_1100_1xx_ldm(opcode) \
  { \
    uint32_t Rn = (opcode >> 24) & 0x7;\
    uint32_t reg_list = (opcode >> 16) & 0xff;\
    bool wback = (reg_list & (1<<Rn)) == 0;\
    uint32_t Rn_val = get_reg(Rn);\
    std::cout << "LDM R" << Rn << (wback?" {":"! {") << std::bitset<8>{reg_list} << "}" << std::endl;\
    for (int i = 0; i < 8; i++) {\
      if ((reg_list & (1<<i)) == 0) continue;\
      set_reg(i, co_await m_mpu_bus_interface.read_word(Rn_val));\
      Rn_val += 4;\
    }\
    if (wback) set_reg(Rn, Rn_val);\
  }

#define OPCODE_1101_xxx_branch(opcode) \
  { \
    uint32_t cond = (opcode >> 24) & 0xf; \
    uint32_t imm8 = (opcode >> 16) & 0xFF; \
    if (imm8 >= 128) imm8 -= 256; \
    uint32_t addr = PC() + imm8*2 + 4;\
    bool jump;\
    switch (cond) {\
      case Cond::EQ: \
        jump = (m_APSR & APSR::ZERO); \
        std::cout << "BEQ "; break;\
      case Cond::NE: \
        jump = !(m_APSR & APSR::ZERO); \
        std::cout << "BNE "; break;\
      case Cond::CS: \
        jump = (m_APSR & APSR::CARRY); \
        std::cout << "BCS "; break;\
      case Cond::CC: \
        jump = !(m_APSR & APSR::CARRY); \
        std::cout << "BCC "; break;\
      case Cond::MI: \
        jump = (m_APSR & APSR::NEGATIVE); \
        std::cout << "BMI "; break;\
      case Cond::PL: \
        jump = !(m_APSR & APSR::NEGATIVE); \
        std::cout << "BPL "; break;\
      case Cond::VS: \
        jump = (m_APSR & APSR::OVERFLOW); \
        std::cout << "BVS "; break;\
      case Cond::VC: \
        jump = !(m_APSR & APSR::OVERFLOW); \
        std::cout << "BVC "; break;\
      case Cond::HI: \
        jump = ((m_APSR & APSR::CARRY) && !(m_APSR & APSR::ZERO)); \
        std::cout << "BHI "; break;\
      case Cond::LS: \
        jump = (!(m_APSR & APSR::CARRY) || (m_APSR & APSR::ZERO)); \
        std::cout << "BLS "; break;\
      case Cond::GE: \
        jump = (!!(m_APSR & APSR::NEGATIVE) == !!(m_APSR & APSR::OVERFLOW)); \
        std::cout << "BGE "; break;\
      case Cond::LT: \
        jump = (!!(m_APSR & APSR::NEGATIVE) != !!(m_APSR & APSR::OVERFLOW)); \
        std::cout << "BLT "; break;\
      case Cond::GT: \
        jump = (!(m_APSR & APSR::ZERO) && (!!(m_APSR & APSR::NEGATIVE) == !!(m_APSR & APSR::OVERFLOW))); \
        std::cout << "BGT "; break;\
      case Cond::LE: \
        jump = ((m_APSR & APSR::ZERO) && (!!(m_APSR & APSR::NEGATIVE) != !!(m_APSR & APSR::OVERFLOW))); \
        std::cout << "BLE "; break;\
      default: throw HardFault{"unrecognised branch instr"}; \
    }\
    std::cout << std::hex << addr << std::dec << " " << std::bitset<4>{m_APSR>>28} << (jump?" y":" n") << std::endl;\
    if (jump) { \
      m_nextPC = addr; /*This isn't right!*/ \
    } \
  }

#define OPCODE_1110_0xx_branch(opcode) \
  { \
    uint32_t imm11 = (opcode >> 15) & 0xffe; \
    if (imm11 >= 2048) imm11 -= 4096; \
    uint32_t addr = PC() + imm11 + 4;\
    std::cout << "B " << std::hex << addr << std::dec << std::endl;\
    m_nextPC = addr; \
  }

#define OPCODE_1111_001_mov_spec(opcode) \
  {\
    uint32_t sysm = opcode & 0xff;\
    uint32_t Rd = (opcode >> 8) & 0xf;\
    uint32_t Rn = (opcode >> 16) & 0xf;\
    switch(opcode & 0x07e0'0000){\
      case 0x0380'0000: /*MSR*/ \
      {\
        switch(sysm){\
          case 0x04:\
          case 0x05:\
          case 0x06:\
            m_APSR = get_reg(Rn) & 0xf800'0000;\
            break;\
          case 0x08: \
          if (CurrentModeIsPrivileged()) {\
            set_MSP(get_reg(Rn) & ~0x3); \
          } break; \
          case 0x09: \
          if (CurrentModeIsPrivileged()) {\
            set_PSP(get_reg(Rn) & ~0x3); \
          } break; \
        }\
      }\
      case 0x03e0'0000: ;/*MRS*/ \
    }\
  }

#define OPCODE_1111_01x_bl(opcode) \
  { \
    if ((opcode & 0x0000'5000) == 0x0000'0000) {\
      OPCODE_1111_001_mov_spec(opcode)\
    } else {\
      uint8_t cond = (opcode >> 24) & 0xf; \
      uint32_t imm11 = (opcode) & 0x7ff; \
      uint32_t imm10 = (opcode >> 16) & 0x3ff; \
      uint32_t I1 = (opcode >> 13) &1;\
      uint32_t I2 = (opcode >> 11) &1;\
      uint32_t imm32 = ((imm11 << 1) | (imm10 << 12) | (I2 << 22) | (I1 << 23)) - (1<<24);\
      uint32_t addr = m_nextPC + imm32;\
      std::cout << "BL " << std::hex << addr << std::dec << std::endl;\
      LR() = m_nextPC;\
      m_nextPC = addr;\
    }\
  }

#define OPCODE_1111_00x_bl(opcode) \
  { \
    if ((opcode & 0x0000'5000) == 0x0000'0000) {\
      OPCODE_1111_001_mov_spec(opcode)\
    } else {\
      uint8_t cond = (opcode >> 24) & 0xf; \
      uint32_t imm11 = (opcode) & 0x7ff; \
      uint32_t imm10 = (opcode >> 16) & 0x3ff; \
      uint32_t I1 = 1^((opcode >> 13) &1);\
      uint32_t I2 = 1^((opcode >> 11) &1);\
      uint32_t imm32 = ((imm11 << 1) | (imm10 << 12) | (I2 << 22) | (I1 << 23));\
      uint32_t addr = m_nextPC + imm32;\
      std::cout << "BL " << std::hex << addr << std::dec << std::endl;\
      LR() = m_nextPC;\
      m_nextPC = addr;\
    }\
  }

#define ENUM_OPCODES(o) \
  /*0000_000*/ o(0b0000'000, OPCODE_0000_0xx_lsl) \
  /*0000_001*/ o(0b0000'001, OPCODE_0000_0xx_lsl) \
  /*0000_010*/ o(0b0000'010, OPCODE_0000_0xx_lsl) \
  /*0000_011*/ o(0b0000'011, OPCODE_0000_0xx_lsl) \
  /*0000_100*/ o(0b0000'100, OPCODE_0000_1xx_lsr) \
  /*0000_101*/ o(0b0000'101, OPCODE_0000_1xx_lsr) \
  /*0000_110*/ o(0b0000'110, OPCODE_0000_1xx_lsr) \
  /*0000_111*/ o(0b0000'111, OPCODE_0000_1xx_lsr) \
  /*0001_000*/ o(0b0001'000, OPCODE_UNDEFINED) \
  /*0001_001*/ o(0b0001'001, OPCODE_UNDEFINED) \
  /*0001_010*/ o(0b0001'010, OPCODE_UNDEFINED) \
  /*0001_011*/ o(0b0001'011, OPCODE_UNDEFINED) \
  /*0001_100*/ o(0b0001'100, OPCODE_0001_100_add) \
  /*0001_101*/ o(0b0001'101, OPCODE_UNDEFINED) \
  /*0001_110*/ o(0b0001'110, OPCODE_0001_110_add) \
  /*0001_111*/ o(0b0001'111, OPCODE_0001_111_sub) \
  /*0010_000*/ o(0b0010'000, OPCODE_0010_0xx_mov) \
  /*0010_001*/ o(0b0010'001, OPCODE_0010_0xx_mov) \
  /*0010_010*/ o(0b0010'010, OPCODE_0010_0xx_mov) \
  /*0010_011*/ o(0b0010'011, OPCODE_0010_0xx_mov) \
  /*0010_100*/ o(0b0010'100, OPCODE_0010_1xx_cmp) \
  /*0010_101*/ o(0b0010'101, OPCODE_0010_1xx_cmp) \
  /*0010_110*/ o(0b0010'110, OPCODE_0010_1xx_cmp) \
  /*0010_111*/ o(0b0010'111, OPCODE_0010_1xx_cmp) \
  /*0011_000*/ o(0b0011'000, OPCODE_0011_0xx_add) \
  /*0011_001*/ o(0b0011'001, OPCODE_0011_0xx_add) \
  /*0011_010*/ o(0b0011'010, OPCODE_0011_0xx_add) \
  /*0011_011*/ o(0b0011'011, OPCODE_0011_0xx_add) \
  /*0011_100*/ o(0b0011'100, OPCODE_0011_1xx_sub) \
  /*0011_101*/ o(0b0011'101, OPCODE_0011_1xx_sub) \
  /*0011_110*/ o(0b0011'110, OPCODE_0011_1xx_sub) \
  /*0011_111*/ o(0b0011'111, OPCODE_0011_1xx_sub) \
  /*0100_000*/ o(0b0100'000, OPCODE_0100_00x_arith) \
  /*0100_011*/ o(0b0100'001, OPCODE_0100_00x_arith) \
  /*0100_010*/ o(0b0100'010, OPCODE_UNDEFINED) \
  /*0100_001*/ o(0b0100'011, OPCODE_0100_011_mov) \
  /*0100_100*/ o(0b0100'100, OPCODE_0100_1xx_load) \
  /*0100_101*/ o(0b0100'101, OPCODE_0100_1xx_load) \
  /*0100_110*/ o(0b0100'110, OPCODE_0100_1xx_load) \
  /*0100_111*/ o(0b0100'111, OPCODE_0100_1xx_load) \
  /*0101_000*/ o(0b0101'000, OPCODE_0101_000_store) \
  /*0101_001*/ o(0b0101'001, OPCODE_UNDEFINED) \
  /*0101_010*/ o(0b0101'010, OPCODE_UNDEFINED) \
  /*0101_011*/ o(0b0101'011, OPCODE_UNDEFINED) \
  /*0101_100*/ o(0b0101'100, OPCODE_UNDEFINED) \
  /*0101_101*/ o(0b0101'101, OPCODE_UNDEFINED) \
  /*0101_110*/ o(0b0101'110, OPCODE_UNDEFINED) \
  /*0101_111*/ o(0b0101'111, OPCODE_UNDEFINED) \
  /*0110_000*/ o(0b0110'000, OPCODE_0110_0xx_store) \
  /*0110_001*/ o(0b0110'001, OPCODE_0110_0xx_store) \
  /*0110_010*/ o(0b0110'010, OPCODE_0110_0xx_store) \
  /*0110_011*/ o(0b0110'011, OPCODE_0110_0xx_store) \
  /*0110_100*/ o(0b0110'100, OPCODE_0110_1xx_load) \
  /*0110_101*/ o(0b0110'101, OPCODE_0110_1xx_load) \
  /*0110_110*/ o(0b0110'110, OPCODE_0110_1xx_load) \
  /*0110_111*/ o(0b0110'111, OPCODE_0110_1xx_load) \
  /*0111_000*/ o(0b0111'000, OPCODE_0111_0xx_strb) \
  /*0111_001*/ o(0b0111'001, OPCODE_0111_0xx_strb) \
  /*0111_010*/ o(0b0111'010, OPCODE_0111_0xx_strb) \
  /*0111_011*/ o(0b0111'011, OPCODE_0111_0xx_strb) \
  /*0111_100*/ o(0b0111'100, OPCODE_0111_1xx_ldrb) \
  /*0111_101*/ o(0b0111'101, OPCODE_0111_1xx_ldrb) \
  /*0111_110*/ o(0b0111'110, OPCODE_0111_1xx_ldrb) \
  /*0111_111*/ o(0b0111'111, OPCODE_0111_1xx_ldrb) \
  /*1000_000*/ o(0b1000'000, OPCODE_1000_0xx_strh) \
  /*1000_001*/ o(0b1000'001, OPCODE_1000_0xx_strh) \
  /*1000_010*/ o(0b1000'010, OPCODE_1000_0xx_strh) \
  /*1000_011*/ o(0b1000'011, OPCODE_1000_0xx_strh) \
  /*1000_100*/ o(0b1000'100, OPCODE_1000_1xx_ldrh) \
  /*1000_101*/ o(0b1000'101, OPCODE_1000_1xx_ldrh) \
  /*1000_110*/ o(0b1000'110, OPCODE_1000_1xx_ldrh) \
  /*1000_111*/ o(0b1000'111, OPCODE_1000_1xx_ldrh) \
  /*1001_000*/ o(0b1001'000, OPCODE_1001_0xx_str) \
  /*1001_001*/ o(0b1001'001, OPCODE_1001_0xx_str) \
  /*1001_010*/ o(0b1001'010, OPCODE_1001_0xx_str) \
  /*1001_011*/ o(0b1001'011, OPCODE_1001_0xx_str) \
  /*1001_100*/ o(0b1001'100, OPCODE_1001_1xx_ldr) \
  /*1001_101*/ o(0b1001'101, OPCODE_1001_1xx_ldr) \
  /*1001_110*/ o(0b1001'110, OPCODE_1001_1xx_ldr) \
  /*1001_111*/ o(0b1001'111, OPCODE_1001_1xx_ldr) \
  /*1010_000*/ o(0b1010'000, OPCODE_1010_0xx_add) \
  /*1010_001*/ o(0b1010'001, OPCODE_1010_0xx_add) \
  /*1010_010*/ o(0b1010'010, OPCODE_1010_0xx_add) \
  /*1010_011*/ o(0b1010'011, OPCODE_1010_0xx_add) \
  /*1010_100*/ o(0b1010'100, OPCODE_1010_1xx_spadd) \
  /*1010_101*/ o(0b1010'101, OPCODE_1010_1xx_spadd) \
  /*1010_110*/ o(0b1010'110, OPCODE_1010_1xx_spadd) \
  /*1010_111*/ o(0b1010'111, OPCODE_1010_1xx_spadd) \
  /*1011_000*/ o(0b1011'000, OPCODE_1011_000_sp) \
  /*1011_001*/ o(0b1011'001, OPCODE_1011_001_ext) \
  /*1011_010*/ o(0b1011'010, OPCODE_1011_010_pushm) \
  /*1011_011*/ o(0b1011'011, OPCODE_UNDEFINED) \
  /*1011_100*/ o(0b1011'100, OPCODE_UNDEFINED) \
  /*1011_101*/ o(0b1011'101, OPCODE_1011_101_rev) \
  /*1011_110*/ o(0b1011'110, OPCODE_1011_110_popm) \
  /*1011_111*/ o(0b1011'111, OPCODE_1011_111_misc) \
  /*1100_000*/ o(0b1100'000, OPCODE_UNDEFINED) \
  /*1100_001*/ o(0b1100'001, OPCODE_UNDEFINED) \
  /*1100_010*/ o(0b1100'010, OPCODE_UNDEFINED) \
  /*1100_011*/ o(0b1100'011, OPCODE_UNDEFINED) \
  /*1100_100*/ o(0b1100'100, OPCODE_1100_1xx_ldm) \
  /*1100_101*/ o(0b1100'101, OPCODE_1100_1xx_ldm) \
  /*1100_110*/ o(0b1100'110, OPCODE_1100_1xx_ldm) \
  /*1100_111*/ o(0b1100'111, OPCODE_1100_1xx_ldm) \
  /*1101_000*/ o(0b1101'000, OPCODE_1101_xxx_branch) \
  /*1101_001*/ o(0b1101'001, OPCODE_1101_xxx_branch) \
  /*1101_010*/ o(0b1101'010, OPCODE_1101_xxx_branch) \
  /*1101_011*/ o(0b1101'011, OPCODE_1101_xxx_branch) \
  /*1101_100*/ o(0b1101'100, OPCODE_1101_xxx_branch) \
  /*1101_101*/ o(0b1101'101, OPCODE_1101_xxx_branch) \
  /*1101_110*/ o(0b1101'110, OPCODE_1101_xxx_branch) \
  /*1101_111*/ o(0b1101'111, OPCODE_1101_xxx_branch) \
  /*1110_000*/ o(0b1110'000, OPCODE_1110_0xx_branch) \
  /*1110_001*/ o(0b1110'001, OPCODE_1110_0xx_branch) \
  /*1110_010*/ o(0b1110'010, OPCODE_1110_0xx_branch) \
  /*1110_011*/ o(0b1110'011, OPCODE_1110_0xx_branch) \
  /*1110_100*/ o(0b1110'100, OPCODE_UNDEFINED) \
  /*1110_101*/ o(0b1110'101, OPCODE_UNDEFINED) \
  /*1110_110*/ o(0b1110'110, OPCODE_UNDEFINED) \
  /*1110_111*/ o(0b1110'111, OPCODE_UNDEFINED) \
  /*1111_000*/ o(0b1111'000, OPCODE_1111_00x_bl) \
  /*1111_001*/ o(0b1111'001, OPCODE_1111_00x_bl) \
  /*1111_010*/ o(0b1111'010, OPCODE_1111_01x_bl) \
  /*1111_011*/ o(0b1111'011, OPCODE_1111_01x_bl) \
  /*1111_100*/ o(0b1111'100, OPCODE_UNDEFINED) \
  /*1111_101*/ o(0b1111'101, OPCODE_UNDEFINED) \
  /*1111_110*/ o(0b1111'110, OPCODE_UNDEFINED) \
  /*1111_111*/ o(0b1111'111, OPCODE_UNDEFINED)


Awaitable<void> ARMv6MCore::exec_instr(uint32_t instr)
{
  #define OPCODE(prefix, fun) \
    case prefix: \
      fun(instr); \
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
      out = 0;
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