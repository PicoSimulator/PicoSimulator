#pragma once

/**
 *
 * Each OPCODE macro takes 4 parameters:
 * - opcode: the opcode to match
 * - do_disasm: a macro that takes a block statement as a parameter to execute when disassembling
 * - do_exec: a macro that takes a two block statement as parameters to execute when executing
 *   - the first block is executed in the normal execution path and modifies the core state
 *   - the second block is executed in the trace path and can be used to print debug information
 * - core: the core object to operate on
*/

#define FMT_op16(opcode) std::hex << std::setw(4) << std::setfill('0') << ((opcode >> 16) & 0xffff) << "      " << std::dec
#define FMT_op32(opcode) std::hex << std::setw(4) << std::setfill('0') << ((opcode >> 16) & 0xffff) << " " << (opcode & 0xffff) << " " << std::dec

#define FMT_dis_RdRmImm5(mnem, Rd, Rm, imm5) (mnem) << " " << FMT_reg((Rd)) << ", " << FMT_reg((Rm)) << ", #" << (imm5)
#define FMT_dis_RdnRm(mnem, Rdn, Rm) (mnem) << " " << FMT_reg((Rdn)) << ", " << FMT_reg((Rm))
#define FMT_dis_RnRd(mnem, Rn, Rd) (mnem) << " " << FMT_reg((Rn)) << ", " << FMT_reg((Rd))
#define FMT_dis_RnRdRm(mnem, Rn, Rd, Rm) (mnem) << " " << FMT_reg((Rn)) << ", " << FMT_reg((Rd)) << ", " << FMT_reg((Rm))
#define FMT_dis_imm8Rdn(mnem, imm8, Rdn) (mnem) << " " << FMT_reg((Rdn)) << ", #" << (imm8)
#define FMT_dis_RdRnImm3(mnem, Rd, Rn, imm3) (mnem) << " " << FMT_reg((Rd)) << ", " << FMT_reg((Rn)) << ", #" << (imm3)
#define FMT_dis_Rm(mnem, Rm) (mnem) << " " << FMT_reg((Rm)) 

#define FMT_reg(r) "R" << FMT_dec(r)
#define FMT_hex(v) std::hex << v << std::dec
#define FMT_hexw(v, w) std::hex << std::setw(w) << std::setfill('0') << ((v) & ((((uint64_t)1)<<(4*(w)))-1)) << std::dec
#define FMT_dec(v) std::dec << v
#define FMT_flags(f) ((f) & APSR::NEGATIVE ? "N" : "n") << ((f) & APSR::ZERO ? "Z" : "z") << ((f) & APSR::CARRY ? "C" : "c") << ((f) & APSR::OVERFLOW ? "V" : "v")

#define OPCODE_UNDEFINED(opcode, do_disasm, do_exec, do_trace, core) \
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

#define OPCODE_0000_0xx_lsl(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x07;\
    uint32_t Rm = (opcode >> 19) & 0x07;\
    uint32_t imm5 = (opcode >> 22) & 0x1f;\
    do_disasm({ \
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RdRmImm5("LSL", Rd, Rm, imm5)\
                << std::endl;\
    })\
    do_exec({ \
      uint32_t Rm_val = get_reg(Rm); \
      bool carry_out = Rm_val & (1<<(32-imm5));\
      if (imm5 != 0) {\
        Rm_val <<= imm5;\
      }\
      core->set_reg(Rd, Rm_val);\
      SETFLAGS_NZC(Rm_val, carry_out);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val)  \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl; \
      })\
    })\
  }

#define OPCODE_0000_1xx_lsr(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t Rd = (opcode >> 16) & 0x07;\
    uint32_t Rm = (opcode >> 19) & 0x07;\
    uint32_t imm5 = (opcode >> 22) & 0x1f;\
    do_disasm({ \
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RdRmImm5("LSR", Rd, Rm, imm5)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rm_val = core->get_reg(Rm);\
      bool carry_out = false; \
      if (imm5 != 0) {\
        carry_out = Rm_val & (1 << (imm5-1));\
        Rm_val >>= imm5;\
      }\
      core->set_reg(Rd, Rm_val);\
      SETFLAGS_NZC(Rm_val, carry_out);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val)  \
                  << ";FLAGS=" << FMT_flags(m_APSR)\
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0001_0xx_asr(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t Rd = (opcode >> 16) & 0x07;\
    uint32_t Rm = (opcode >> 19) & 0x07;\
    uint32_t imm5 = (opcode >> 22) & 0x1f;\
    do_disasm({ \
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RdRmImm5("ASR", Rd, Rm, imm5)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rm_val = core->get_reg(Rm);\
      bool carry_out = Rm_val & (1<<imm5);\
      if (imm5 != 0) {\
        Rm_val >>= imm5;\
        Rm_val = SignExtend(Rm_val, 32 - imm5); \
      }\
      core->set_reg(Rd, Rm_val);\
      SETFLAGS_NZC(Rm_val, carry_out);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val)  \
                  << ";FLAGS=" << FMT_flags(m_APSR)\
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0001_100_add(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t Rm = (opcode >> 22) & 0x7;\
    do_disasm({ \
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RnRdRm("ADD", Rd, Rn, Rm)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = core->get_reg(Rn);\
      uint32_t Rm_val = core->get_reg(Rm);\
      auto _res = AddWithCarry(Rn_val, Rm_val, 0);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      core->set_reg(Rd, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val)  \
                  << ";FLAGS=" << FMT_flags(m_APSR)\
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0001_101_sub(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t Rm = (opcode >> 22) & 0x7;\
    do_disasm({\
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RnRdRm("SUB", Rd, Rn, Rm)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      uint32_t Rm_val = get_reg(Rm);\
      auto _res = AddWithCarry(Rn_val, ~Rm_val, 1);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rd, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(result)  \
                  << ";FLAGS=" << FMT_flags(m_APSR)\
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0001_110_add(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t imm3 = (opcode >> 22) & 0x7;\
    do_disasm({\
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RdRnImm3("ADD", Rd, Rn, imm3)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      auto _res = AddWithCarry(Rn_val, imm3, 0);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rd, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(result)  \
                  << ";FLAGS=" << FMT_flags(m_APSR)\
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0001_111_sub(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rn = (opcode >> 19) & 0x7;\
    uint32_t imm3 = (opcode >> 22) & 0x7;\
    do_disasm({\
      std::cout << FMT_op16(opcode)\
                << FMT_dis_RdRnImm3("SUB", Rd, Rn, imm3)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      auto _res = AddWithCarry(Rn_val, ~imm3, 1);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rd, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(result)  \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl;\
      })\
    })\
  }



#define OPCODE_0010_0xx_mov(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t Rd = (opcode >> 24) & 0x07;\
    do_disasm({\
      std::cout << FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("MOV", imm8, Rd)\
                << std::endl;\
    })\
    do_exec({\
      set_reg(Rd, imm8);\
      if (imm8) {\
        m_APSR &= ~APSR::ZERO;\
      } else {\
        m_APSR |= APSR::ZERO;\
      }\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(imm8)  \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0010_1xx_cmp(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    uint32_t simm32 = ~imm8;\
    uint32_t Rn = (opcode >> 24) & 0x07;\
    do_disasm({\
      std::cout << FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("CMP", imm8, Rn)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      auto _res = AddWithCarry(Rn_val, simm32, 1);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << "result=0x" << FMT_hex(result) \
                  << ";Rn_val=0x" << FMT_hex(Rn_val) \
                  << ";simm32=0x" << FMT_hex(simm32) \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0011_0xx_add(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rdn = (opcode >> 24) & 0x7;\
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    do_disasm({\
      std::cout << FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("ADD", imm8, Rdn)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rdn);\
      auto _res = AddWithCarry(Rn_val, imm8, 0);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rdn, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(result)  \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0011_1xx_sub(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rdn = (opcode >> 24) & 0x7;\
    uint32_t imm8 = (opcode >> 16) & 0xff;\
    do_disasm({\
      std::cout << FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("SUB", imm8, Rdn)\
                << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rdn);\
      auto _res = AddWithCarry(Rn_val, ~imm8, 1);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rdn, result);\
      SETFLAGS_NZCV(result, carry, overflow);\
      do_trace({\
        std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(result)  \
                  << ";FLAGS=" << FMT_flags(m_APSR) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0100_00x_arith(opcode, do_disasm, do_exec, do_trace, core) \
{\
  uint8_t sub_opcode = (opcode >> 22) & 0xf;\
  switch(sub_opcode) {\
    case 0b0000:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("AND", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        Rdn_val &= Rm_val; \
        set_reg(Rdn, Rdn_val); \
        SETFLAGS_NZ(Rdn_val);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
      })\
      })\
    } break; \
    case 0b0001:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("EOR", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        Rdn_val ^= Rm_val; \
        set_reg(Rdn, Rdn_val); \
        SETFLAGS_NZ(Rdn_val);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break; \
    case 0b0010:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("LSL", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool carry = Rdn_val&(1<<(32-Rm));\
        Rdn_val <<= Rm_val; \
        set_reg(Rdn, Rdn_val); \
        SETFLAGS_NZC(Rdn_val, carry);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break; \
    case 0b0011:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("LSR", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool carry = Rdn_val&(1<<(Rm_val-1));\
        Rdn_val >>= Rm_val; \
        set_reg(Rdn, Rdn_val); \
        SETFLAGS_NZC(Rdn_val, carry);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break; \
    case 0b0100:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("ASR", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool carry = Rdn_val&(1<<(Rm_val-1));\
        Rdn_val >>= Rm_val; \
        set_reg(Rdn, Rdn_val); \
        SETFLAGS_NZC(Rdn_val, carry);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool carry = Rdn_val&(1<<(Rm_val-1));\
        Rm_val >>= Rm_val;\
        Rm_val = SignExtend(Rm_val, 32 - Rm_val); \
        core->set_reg(Rdn, Rdn_val);\
        SETFLAGS_NZC(Rdn_val, carry);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR)\
                    << std::endl;\
        })\
      })\
    } break; \
    case 0b0101:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("ADC", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool cin = m_APSR & APSR::CARRY; \
        auto _res = AddWithCarry(Rdn_val, Rm_val, cin); \
        auto result = std::get<0>(_res);\
        auto cout = std::get<1>(_res);\
        auto v = std::get<2>(_res);\
        set_reg(Rdn, result); \
        SETFLAGS_NZCV(result, cout, v);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(result)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
      })\
      })\
    } break; \
    case 0b0110:{\
      uint32_t Rdn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("SBC", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        bool cin = m_APSR & APSR::CARRY; \
        auto _res = AddWithCarry(Rdn_val, ~Rm_val, cin); \
        auto result = std::get<0>(_res);\
        auto cout = std::get<1>(_res);\
        auto v = std::get<2>(_res);\
        set_reg(Rdn, result); \
        SETFLAGS_NZCV(result, cout, v);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(result)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break; \
    case 0b1000:{\
      uint32_t Rn = (opcode >> 16) & 0x07;\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("TST", Rn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rn_val = get_reg(Rn);\
        uint32_t Rm_val = get_reg(Rm);\
        uint32_t result = Rn_val & Rm_val;\
        SETFLAGS_NZ(result);\
        do_trace({\
          std::cout << "FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1001:{\
      uint32_t Rd = (opcode >> 16) & 0x07;\
      uint32_t Rn = (opcode >> 19) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("RSB", Rd, Rn) << std::endl; })\
      do_exec({\
        uint32_t Rn_val = get_reg(Rn);\
        auto _res = AddWithCarry(~Rn_val, 0, 1);\
        auto result = std::get<0>(_res);\
        auto carry = std::get<1>(_res);\
        auto overflow = std::get<2>(_res);\
        set_reg(Rd, result); \
        SETFLAGS_NZCV(result, carry, overflow);\
        do_trace({\
          std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(result)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1010:{\
      uint32_t Rm = (opcode >> 19) & 0x07;\
      uint32_t Rn = (opcode >> 16) & 0x07;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("CMP", Rn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rm_val = get_reg(Rm);\
        uint32_t Rn_val = get_reg(Rn);\
        auto _res = AddWithCarry(Rn_val, ~Rm_val, 1);\
        auto result = std::get<0>(_res);\
        auto carry = std::get<1>(_res);\
        auto overflow = std::get<2>(_res);\
        SETFLAGS_NZCV(result, carry, overflow);\
        do_trace({\
          std::cout << "FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1100:{\
      uint32_t Rdn = (opcode >> 16) & 0x7;\
      uint32_t Rm = (opcode >> 19) & 0x7;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("ORR", Rdn, Rm) << std::endl; })\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        Rdn_val |= Rm_val;\
        set_reg(Rdn, Rdn_val);\
        SETFLAGS_NZ(Rdn_val);\
        do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(Rdn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1101:{\
      uint32_t Rdm = (opcode >> 16) & 0x7;\
      uint32_t Rn = (opcode >> 19) & 0x7;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("MULS", Rdm, Rn) << std::endl; })\
      do_exec({\
        uint32_t Rdm_val = get_reg(Rdm);\
        uint32_t Rn_val = get_reg(Rn);\
        Rdm_val *= Rn_val;\
        set_reg(Rdm, Rdm_val);\
        SETFLAGS_NZ(Rdm_val);\
        do_trace({\
          std::cout << FMT_reg(Rdm) << ":=0x" << FMT_hex(Rdm_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1110:{\
      uint32_t Rd = (opcode >> 16) & 0x7;\
      uint32_t Rn = (opcode >> 19) & 0x7;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("BIC", Rd, Rn) << std::endl; })\
      do_exec({\
        uint32_t Rd_val = get_reg(Rd);\
        uint32_t Rn_val = get_reg(Rn);\
        Rd_val &= ~Rn_val;\
        set_reg(Rd, Rd_val);\
        SETFLAGS_NZ(Rd_val);\
        do_trace({\
          std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rd_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    case 0b1111:{\
      uint32_t Rd = (opcode >> 16) & 0x7;\
      uint32_t Rn = (opcode >> 19) & 0x7;\
      do_disasm({std::cout << FMT_op16(opcode) << FMT_dis_RdnRm("MVN", Rd, Rn) << std::endl; })\
      do_exec({\
        uint32_t Rn_val = ~get_reg(Rn);\
        set_reg(Rd, Rn_val);\
        SETFLAGS_NZC(Rn_val, false);\
        do_trace({\
          std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rn_val)  \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    default: throw HardFault{"Unknown arith instr"};\
  }\
}

#define OPCODE_0100_010_special(opcode, do_disasm, do_exec, do_trace, core) \
{\
  switch(opcode & 0x03c0'0000) {\
    case 0x0000'0000:\
    case 0x0040'0000:\
    case 0x0080'0000:\
    case 0x00c0'0000:\
    {\
      uint32_t Rdn = (opcode >> 16) & 0x07 | (opcode >> 20) & 0x08;\
      uint32_t Rm = (opcode >> 19) & 0x0f;\
      /* do_disasm({\
        std::cout << FMT_op16(opcode) \
                  << FMT_dis_RdnRm("ADD", Rdn, Rm) \
                  << std::cout;\
      }) */\
      do_exec({\
        uint32_t Rdn_val = get_reg(Rdn);\
        uint32_t Rm_val = get_reg(Rm);\
        auto _res = AddWithCarry(Rdn_val, Rm_val, 0);\
        auto result = std::get<0>(_res);\
        auto carry = std::get<1>(_res);\
        auto overflow = std::get<2>(_res);\
        set_reg(Rdn, result);\
        SETFLAGS_NZCV(result, carry, overflow);\
        /* do_trace({\
          std::cout << FMT_reg(Rdn) << ":=0x" << FMT_hex(result) \
                    << ";FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        }) */\
      })\
    } break;\
    case 0x0140'0000: \
    {\
      uint32_t Rm = (opcode >> 19) & 0xf;\
      uint32_t Rn = (opcode >> 16) & 0x7 | (opcode >> 19) & 0x8;\
      do_disasm({\
        std::cout << FMT_op16(opcode) \
                  << FMT_dis_RdnRm("CMP", Rn, Rm) \
                  << std::endl; \
      })\
      do_exec({\
        uint32_t Rm_val = get_reg(Rm);\
        uint32_t Rn_val = get_reg(Rn);\
        auto _res = AddWithCarry(Rn_val, ~Rm_val, 1);\
        auto result = std::get<0>(_res);\
        auto carry = std::get<1>(_res);\
        auto overflow = std::get<2>(_res);\
        SETFLAGS_NZCV(result, carry, overflow);\
        do_trace({\
          std::cout << "FLAGS=" << FMT_flags(m_APSR) \
                    << std::endl;\
        })\
      })\
    } break;\
    default: throw HardFault{}; \
  }\
}

#define OPCODE_0100_011_mov(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    if ((opcode & 1<<24) == 0){\
      uint32_t Rd = ((opcode >> 16) & 0x7) | ((opcode >> 20) & 0x08) ; \
      uint32_t Rm = (opcode >> 19) & 0xf; \
      do_disasm({\
        std::cout << FMT_op16(opcode) \
                  << FMT_dis_RdnRm("MOV", Rd, Rm) \
                  << std::endl; \
      })\
      do_exec({\
        uint32_t RegVal = get_reg(Rm);\
        if (Rd == 15) {\
          ALUWritePC(RegVal);\
          do_trace({\
            std::cout << "PC:=0x" << std::hex << RegVal\
                      << std::endl;\
          })\
        } else {\
          set_reg(Rd, RegVal); \
          SETFLAGS_NZ(RegVal);\
          do_trace({\
            std::cout << FMT_reg(Rd) << ":=0x" << std::hex << RegVal\
                      << ";FLAGS=" << FMT_flags(m_APSR) \
                      << std::endl;\
          })\
        }\
      })\
      } else if ((opcode & 1<<23) == 0) {\
        uint32_t Rm = (opcode>>19) & 0xf;\
        do_disasm({\
          std::cout << FMT_op16(opcode) \
                    << FMT_dis_Rm("BX", Rm) \
                    << std::endl; \
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          BXWritePC(Rm_val); \
          do_trace({\
            std::cout << "PC:=0x" << std::hex << Rm_val\
                      << std::endl;\
          })\
        })\
      } else {\
        uint32_t Rm = (opcode>>19) & 0xf;\
        do_disasm({\
          std::cout << FMT_op16(opcode) \
                    << FMT_dis_Rm("BLX", Rm) \
                    << std::endl; \
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          LR() = m_nextPC;\
          BLXWritePC(Rm_val); \
          do_trace({\
            std::cout << "LR:=0x" << std::hex << LR()\
                      << ";PC:=0x" << std::hex << Rm_val\
                      << std::endl;\
          })\
        })\
      }\
  }

#define OPCODE_0100_1xx_load(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    do_disasm({\
      /* std::cout FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("LDR", imm8, Rt) \
                << std::endl; */\
      std::cout << FMT_op16(opcode) << "LDR " << FMT_reg(Rt) << ", [PC, #" << imm32 << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = (PC() & ~3) + imm32 + 4; \
      uint32_t data = co_await m_bus_interface.read_word(addr); \
      set_reg(Rt, data); \
      do_trace({\
          std::cout << FMT_reg(Rt) << ":=MemU[" << FMT_hexw(addr, 8) << ",4]=" \
                    << FMT_hex(data) \
                    << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_000_store(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      /* std::cout FMT_op16(opcode) \
                << FMT_dis_imm8Rdn("LDR", imm8, Rt) \
                << std::endl; */\
      std::cout << FMT_op16(opcode) << "STR " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_word(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",4]:=" << data \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_001_strh(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STRH " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_halfword(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",2]:=" << (data&0xffff) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_010_strb(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STRB " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_byte(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",1]:=0x" << std::hex << (data&0xff) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_011_ldrsb(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRSB " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = co_await m_bus_interface.read_byte(addr); \
      data = SignExtend(data, 8);\
      set_reg(Rt, data);\
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemS[" << std::hex << addr << ",1]=0x" << std::hex << (data) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_100_ldr(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDR " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = co_await m_bus_interface.read_word(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",4]=0x" << std::hex << (data) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_101_ldrh(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRH " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = co_await m_bus_interface.read_halfword(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",2]=0x" << std::hex << (data) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_110_ldrb(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRB " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = co_await m_bus_interface.read_byte(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",1]=0x" << std::hex << (data) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0101_111_ldrsh(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rm = (instr >> 22) & 0x7; \
    uint32_t Rn = (instr >> 19) & 0x7; \
    uint32_t Rt = (instr >> 16) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRSH " << FMT_reg(Rt) << ", [" << FMT_reg(Rm) << ", " << FMT_reg(Rn) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + get_reg(Rm); \
      uint32_t data = co_await m_bus_interface.read_halfword(addr); \
      data = SignExtend(data, 16);\
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemS[" << std::hex << addr << ",2]=0x" << FMT_hexw(data, 8) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0110_0xx_store(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 20) & 0x7C; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STR " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm32; \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_word(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",4]:=" << data \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0110_1xx_load(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 20) & 0x7C; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDR " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << imm32 << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm32; \
      uint32_t data = co_await m_bus_interface.read_word(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",4]=0x" << FMT_hexw(data, 8) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0111_0xx_strb(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm5 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STRB " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << FMT_dec(imm5) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm5; \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_byte(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",1]:=" << data \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_0111_1xx_ldrb(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 22) & 0x1F; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRB " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm32; \
      uint32_t data = co_await m_bus_interface.read_byte(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",1]=0x" << FMT_hexw(data, 2) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1000_0xx_strh(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 21) & 0x3E; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STRH " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm32; \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_halfword(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",2]:=0x" << FMT_hexw(data, 4) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1000_1xx_ldrh(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 21) & 0x3E; \
    uint32_t Rt = (opcode >> 16) & 0x7; \
    uint32_t Rn = (opcode >> 19) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDRH " << FMT_reg(Rt) << ", [" << FMT_reg(Rn) << ", #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = get_reg(Rn) + imm32; \
      uint32_t data = co_await m_bus_interface.read_halfword(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",2]=0x" << FMT_hexw(data, 4) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1001_0xx_str(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STR " << FMT_reg(Rt) << ", [SP, #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = SP() + imm32; \
      uint32_t data = get_reg(Rt); \
      co_await m_bus_interface.write_word(addr, data); \
      do_trace({\
        std::cout << "MemU[" << std::hex << addr << ",4]:=" << data \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1001_1xx_ldr(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rt = (opcode >> 24) & 0x7; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDR " << FMT_reg(Rt) << ", [SP, #" << FMT_dec(imm32) << "]" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = SP() + imm32; \
      uint32_t data = co_await m_bus_interface.read_word(addr); \
      set_reg(Rt, data); \
      do_trace({\
        std::cout << FMT_reg(Rt) << ":=MemU[" << std::hex << addr << ",4]=0x" << FMT_hexw(data, 8) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1010_0xx_add(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rd = (opcode >> 24) & 0x07; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "ADD " << FMT_reg(Rd) << ", PC, #" << FMT_dec(imm32) << std::endl;\
    })\
    do_exec({\
      uint32_t addr = (PC() & ~0x03) + imm32 + 4; \
      set_reg(Rd, addr); \
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(addr) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1010_1xx_spadd(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x3FC; \
    uint32_t Rd = (opcode >> 24) & 0x7;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << "ADD " << FMT_reg(Rd) << ", SP, #" << FMT_dec(imm32) << std::endl;\
    })\
    do_exec({\
      auto _res = AddWithCarry(SP(), imm32, 0);\
      auto result = std::get<0>(_res);\
      auto carry = std::get<1>(_res);\
      auto overflow = std::get<2>(_res);\
      set_reg(Rd, result); \
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(result) \
                  << std::endl;\
      })\
    })\
  }

#define OPCODE_1011_000_sp(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm32 = (opcode >> 14) & 0x1fc;\
    switch(opcode & 0x0080'0000){\
      case 0x0000'0000: {\
        do_disasm({\
          std::cout << FMT_op16(opcode) << "ADD SP, #" << FMT_dec(imm32) << std::endl;\
        })\
        do_exec({\
          auto _res = AddWithCarry(SP(), imm32, 0);\
          auto newsp = std::get<0>(_res);\
          auto carry = std::get<1>(_res);\
          auto overflow = std::get<2>(_res);\
          SP() = newsp;\
          do_trace({\
            std::cout << "SP:=0x" << FMT_hex(newsp) \
                      << std::endl;\
          })\
        })\
      } break;\
      case 0x0080'0000: {\
        /* do_disam({\
          std::cout << "SUB SP, #" << FMT_dec(imm32) << std::endl;\
        }) */\
        do_exec({\
          uint32_t imm32 = (opcode >> 14) & 0x1fc;\
          auto _res = AddWithCarry(SP(), ~imm32, 1);\
          auto newsp = std::get<0>(_res);\
          auto carry = std::get<1>(_res);\
          auto overflow = std::get<2>(_res);\
          SP() = newsp;\
          do_trace({\
            std::cout << "SP:=0x" << FMT_hex(newsp) \
                      << std::endl;\
          })\
        })\
      } break;\
      default: throw HardFault{"SPSE something or other"};\
    }\
  }

#define OPCODE_1011_001_ext(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rm = (opcode >> 19) & 0x7;\
    switch(opcode & 0x00c0'0000){\
      case 0x0000'0000: {\
        do_disasm({\
          std::cout << FMT_op16(opcode) << "SXTH " << FMT_reg(Rd) << ", " << FMT_reg(Rm) << std::endl;\
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          Rm_val = SignExtend(Rm_val&0xffff, 16);\
          set_reg(Rd, Rm_val);\
          do_trace({\
            std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val) \
                      << std::endl;\
          })\
        })\
      } break;\
      case 0x0040'0000: {\
        do_disasm({\
          std::cout << FMT_op16(opcode) << "SXTB " << FMT_reg(Rd) << ", " << FMT_reg(Rm) << std::endl;\
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          Rm_val = SignExtend(Rm_val&0xff, 8);\
          set_reg(Rd, Rm_val);\
          do_trace({\
            std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val) \
                      << std::endl;\
          })\
        })\
      } break;\
      case 0x0080'0000: {\
        do_disasm({\
          std::cout << FMT_op16(opcode) << "UXTH " << FMT_reg(Rd) << ", " << FMT_reg(Rm) << std::endl;\
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          Rm_val = Rm_val&0xffff;\
          set_reg(Rd, Rm_val);\
          do_trace({\
            std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val) \
                      << std::endl;\
          })\
        })\
      } break;\
      case 0x00c0'0000: {\
        do_disasm({\
          std::cout << FMT_op16(opcode) << "UXTB " << FMT_reg(Rd) << ", " << FMT_reg(Rm) << std::endl;\
        })\
        do_exec({\
          uint32_t Rm_val = get_reg(Rm);\
          Rm_val = Rm_val&0xff;\
          set_reg(Rd, Rm_val);\
          do_trace({\
            std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val) \
                      << std::endl;\
          })\
        })\
      } break;\
      default: throw HardFault{"Sign Extenison something or other"};\
    }\
  }

#define OPCODE_1011_010_pushm(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t regs_list = (opcode >> 16) & 0xff | (opcode >> 10) & 0x4000;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << "PUSH {" << std::bitset<16>{regs_list} << "}" << std::endl;\
    }) \
    do_exec({\
      uint32_t addr = SP();\
      for (int i = 14; i >= 0; i--) {\
        if ((regs_list & (1<<i)) == 0) continue; \
        uint32_t val = get_reg(i);\
        addr -= 4;\
        co_await m_bus_interface.write_word(addr, val);\
        do_trace({\
          std::cout << "MemU[" << FMT_hexw(addr, 8) << ",4]:=" << FMT_hexw(val, 8) << std::endl;\
        })\
      }\
      SP() = addr;\
      do_trace({\
        std::cout << "SP:=0x" << FMT_hexw(addr, 8) << std::endl;\
      })\
    })\
  }

#define OPCODE_1011_011_pcs(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t im = opcode & 0x0010'0000;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << (im?"CPSIE i":"CPSID i") << std::endl;\
    })\
    do_exec({\
      std::cout << "WARNING CPS INSTRUCTION NOT IMPLEMENTED" << std::endl; \
    })\
  }

#define OPCODE_1011_101_rev(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t Rd = (opcode >> 16) & 0x7;\
    uint32_t Rm = (opcode >> 19) & 0x7;\
    do_disasm({\
      std::cout << FMT_op16(opcode);\
      switch(opcode & 0x01c0'0000){\
        case 0x0000'0000:\
          std::cout << "REV "; break; \
        case 0x0040'0000:\
          std::cout << "REV16 "; break; \
        case 0x00c0'0000:\
          std::cout << "REVSH "; \
        default: \
          throw HardFault{"bad REV instr"}; \
      }\
      std::cout << FMT_reg(Rd) << ", " << FMT_reg(Rm) << std::endl;\
    })\
    do_exec({\
      uint32_t Rm_val = get_reg(Rm);\
      switch(opcode & 0x01c0'0000){\
        case 0x0000'0000:\
          Rm_val = ((Rm_val & 0xff) << 24) | ((Rm_val & 0xff00) << 8) | ((Rm_val & 0xff0000) >> 8) | ((Rm_val & 0xff000000) >> 24);\
          break;\
        case 0x0040'0000:\
          Rm_val = ((Rm_val & 0xffff) << 16) | ((Rm_val & 0xffff0000) >> 8);\
          break;\
        case 0x00c0'0000:\
          Rm_val = ((Rm_val & 0xffff) << 16) | ((Rm_val & 0xffff0000) >> 8);\
          break;\
        default: \
          throw HardFault{"bad REV instr"}; \
      }\
      set_reg(Rd, Rm_val);\
      do_trace({\
        std::cout << FMT_reg(Rd) << ":=0x" << FMT_hex(Rm_val) << std::endl;\
      })\
    })\
  }

#define OPCODE_1011_110_popm(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t regs_list = (opcode >> 16) & 0xff | (opcode >> 9) & 0x8000;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << "POP {" << std::bitset<16>{regs_list} << "}" << std::endl;\
    })\
    do_exec({\
      uint32_t addr = SP();\
      for (int i = 0; i <= 7; i++) {\
        if ((regs_list & (1<<i)) == 0) continue; \
        uint32_t val= co_await m_bus_interface.read_word(addr); \
        set_reg(i, val);\
        do_trace({\
          std::cout << FMT_reg(i) << ":=MemU[" << FMT_hexw(addr, 8) << ",4]=0x" << FMT_hexw(val, 8) << std::endl;\
        })\
        addr += 4;\
      }\
      if (regs_list & (1<<15)) {\
        uint32_t val = co_await m_bus_interface.read_word(addr);\
        LoadWritePC(val);\
        do_trace({\
          std::cout << "PC:=MemU[" << FMT_hexw(addr, 8) << ",4]=0x" << FMT_hexw(val, 8) << std::endl;\
        })\
        addr += 4;\
      }\
      SP() = addr;\
      do_trace({\
        std::cout << "SP:=0x" << FMT_hex(addr) << std::endl;\
      })\
    })\
  }

#define OPCODE_1011_111_misc(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    do_disasm({\
      std::cout << FMT_op16(opcode);\
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
        /*throw HardFault{"BKPT not implemented"};*/\
      }\
    })\
    do_exec({\
      if (opcode & (1<<24)){\
        /*being lazy not checking for opb==0*/\
        uint32_t opa = (opcode >> 20) & 0xf;\
        switch(opa) {\
          case 0b0000: /*NOP*/ \
            break;\
          case 0b0001: /*YIELD*/ \
            break;\
          case 0b0010: /*WFE*/ \
            while (!EventRegistered()) co_await next_tick();\
            ClearEventRegister();\
            break;\
          case 0b0011: /*WFI*/ \
            while (!EventRegistered()) co_await next_tick();\
            ClearEventRegister();\
            break;\
          case 0b0100: /*SEV*/ \
            SendEvent();\
            break;\
          default: throw HardFault{"Undefined opcode"};\
        }\
      } else {\
        /*BKPT*/\
        uint32_t imm8 = (opcode >> 16) & 0xFF; \
        /*throw HardFault{"BKPT not implemented"};*/\
      }\
    })\
  }

#define OPCODE_1100_0xx_stm(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rn = (opcode >> 24) & 0x7;\
    uint32_t reg_list = (opcode >> 16) & 0xff;\
    bool wback = true;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << "STMIA " << FMT_reg(Rn) << (wback?"! {":" {") << std::bitset<8>{reg_list} << "}" << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      for (int i = 0; i < 8; i++) {\
        if ((reg_list & (1<<i)) == 0) continue;\
        uint32_t val = get_reg(i); \
        co_await m_bus_interface.write_word(Rn_val, val);\
        do_trace({\
          std::cout << "MemU[" << FMT_hexw(Rn_val, 8) << ",4]:=0x" << FMT_hexw(val, 8) << std::endl;\
        })\
        Rn_val += 4;\
      }\
      if (wback) {\
        set_reg(Rn, Rn_val);\
        do_trace({\
          std::cout << FMT_reg(Rn) << ":=0x" << FMT_hex(Rn_val) << std::endl;\
        })\
      }\
    })\
  }

#define OPCODE_1100_1xx_ldm(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t Rn = (opcode >> 24) & 0x7;\
    uint32_t reg_list = (opcode >> 16) & 0xff;\
    bool wback = (reg_list & (1<<Rn)) == 0;\
    do_disasm({\
      std::cout << FMT_op16(opcode) << "LDMIA " << FMT_reg(Rn) << (wback?"! {":" {") << std::bitset<8>{reg_list} << "}" << std::endl;\
    })\
    do_exec({\
      uint32_t Rn_val = get_reg(Rn);\
      for (int i = 0; i < 8; i++) {\
        if ((reg_list & (1<<i)) == 0) continue;\
        uint32_t val = co_await m_bus_interface.read_word(Rn_val);\
        set_reg(i, val);\
        do_trace({\
          std::cout << FMT_reg(i) << ":=MemU[" << FMT_hexw(Rn_val, 8) << ",4]=0x" << FMT_hexw(val, 8) << std::endl;\
        })\
        Rn_val += 4;\
      }\
      if (wback) {\
        set_reg(Rn, Rn_val);\
        do_trace({\
          std::cout << FMT_reg(Rn) << ":=0x" << FMT_hex(Rn_val) << std::endl;\
        })\
      }\
    })\
  }

#define OPCODE_1101_xxx_branch(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t cond = (opcode >> 24) & 0xf; \
    uint32_t imm32 = (opcode >> 15) & 0x1FE; \
    if (imm32 >= 256) imm32 -= 512; \
    uint32_t offset = imm32+4;\
    do_disasm({\
      std::cout << FMT_op16(opcode);\
      switch (cond) {\
        case Cond::EQ: std::cout << "BEQ "; break;\
        case Cond::NE: std::cout << "BNE "; break;\
        case Cond::CS: std::cout << "BCS "; break;\
        case Cond::CC: std::cout << "BCC "; break;\
        case Cond::MI: std::cout << "BMI "; break;\
        case Cond::PL: std::cout << "BPL "; break;\
        case Cond::VS: std::cout << "BVS "; break;\
        case Cond::VC: std::cout << "BVC "; break;\
        case Cond::HI: std::cout << "BHI "; break;\
        case Cond::LS: std::cout << "BLS "; break;\
        case Cond::GE: std::cout << "BGE "; break;\
        case Cond::LT: std::cout << "BLT "; break;\
        case Cond::GT: std::cout << "BGT "; break;\
        case Cond::LE: std::cout << "BLE "; break;\
        default: throw HardFault{"unrecognised branch instr"}; \
      }\
      std::cout << "~#" << FMT_dec(int32_t(offset)) << std::endl;\
    })\
    do_exec({\
      uint32_t addr = PC() + offset;\
      bool jump;\
      switch (cond) {\
        case Cond::EQ: \
          jump = (m_APSR & APSR::ZERO); break; \
        case Cond::NE: \
          jump = !(m_APSR & APSR::ZERO); break; \
        case Cond::CS: \
          jump = (m_APSR & APSR::CARRY); break; \
        case Cond::CC: \
          jump = !(m_APSR & APSR::CARRY); break; \
        case Cond::MI: \
          jump = (m_APSR & APSR::NEGATIVE); break; \
        case Cond::PL: \
          jump = !(m_APSR & APSR::NEGATIVE); break; \
        case Cond::VS: \
          jump = (m_APSR & APSR::OVERFLOW); break; \
        case Cond::VC: \
          jump = !(m_APSR & APSR::OVERFLOW); break; \
        case Cond::HI: \
          jump = ((m_APSR & APSR::CARRY) && !(m_APSR & APSR::ZERO)); break; \
        case Cond::LS: \
          jump = (!(m_APSR & APSR::CARRY) || (m_APSR & APSR::ZERO)); break; \
        case Cond::GE: \
          jump = (!!(m_APSR & APSR::NEGATIVE) == !!(m_APSR & APSR::OVERFLOW)); break; \
        case Cond::LT: \
          jump = (!!(m_APSR & APSR::NEGATIVE) != !!(m_APSR & APSR::OVERFLOW)); break; \
        case Cond::GT: \
          jump = (!(m_APSR & APSR::ZERO) && (!!(m_APSR & APSR::NEGATIVE) == !!(m_APSR & APSR::OVERFLOW))); break; \
        case Cond::LE: \
          jump = ((m_APSR & APSR::ZERO) && (!!(m_APSR & APSR::NEGATIVE) != !!(m_APSR & APSR::OVERFLOW))); break; \
        default: throw HardFault{"unrecognised branch instr"}; \
      }\
      if (jump) { \
        m_nextPC = addr; /*This isn't right!*/ \
        BranchWritePC(addr);\
        do_trace({\
          std::cout << "PC:=0x" << FMT_hex(addr) << std::endl;\
        })\
      } \
    })\
  }

#define OPCODE_1110_0xx_branch(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    uint32_t imm11 = (opcode >> 15) & 0xffe; \
    if (imm11 >= 2048) imm11 -= 4096; \
    int32_t offset = imm11 + 4; \
    do_disasm({\
      std::cout << FMT_op16(opcode) << "B ~#" << FMT_dec(offset) << std::endl;\
    })\
    do_exec({\
      uint32_t addr = PC() + offset;\
      m_nextPC = addr; \
      do_trace({\
        std::cout << "PC:=0x" << FMT_hex(addr) << std::endl;\
      })\
    })\
  }

#define OPCODE_1111_001_mov_spec(opcode, do_disasm, do_exec, do_trace, core) \
  {\
    uint32_t sysm = opcode & 0xff;\
    uint32_t Rd = (opcode >> 8) & 0xf;\
    uint32_t Rn = (opcode >> 16) & 0xf;\
    do_disasm({\
      std::cout << FMT_op32(opcode);\
      switch(opcode & 0x07f0'0000){\
        case 0x0380'0000: /*MSR*/ \
        case 0x039'0000: /*MSR*/ \
          std::cout << "MSR "; break;\
        case 0x03b0'0000: /*MISC*/ \
          std::cout << "DMB? "; break;\
        case 0x03e0'0000: /*MRS*/ \
        case 0x03f0'0000: /*MRS*/ \
          std::cout << "MRS "; break;\
        default: throw HardFault{"undefined opcode1"};\
      }\
      switch(sysm){\
        case 0x04: std::cout << "APSR"; break;\
        case 0x05: std::cout << "IAPSR"; break;\
        case 0x06: std::cout << "EAPSR"; break;\
        case 0x08: std::cout << "MSP"; break;\
        case 0x09: std::cout << "PSP"; break;\
        case 0x10: std::cout << "PRIMASK"; break;\
        default: std::cout << "UNKNOWN";\
      }\
      std::cout << ", " << FMT_reg(Rn) << std::endl;\
    })\
    do_exec({\
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
      do_trace({\
        std::cout << "SPECIAL TRACE UNIMPLEMENTED" << std::endl;\
      })\
    })\
  }

#define OPCODE_1111_01x_bl(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    if ((opcode & 0x0000'5000) == 0x0000'0000) {\
      OPCODE_1111_001_mov_spec(opcode, do_disasm, do_exec, do_trace, core)\
    } else {\
      uint8_t cond = (opcode >> 24) & 0xf; \
      uint32_t imm11 = (opcode) & 0x7ff; \
      uint32_t imm10 = (opcode >> 16) & 0x3ff; \
      uint32_t I1 = (opcode >> 13) &1;\
      uint32_t I2 = (opcode >> 11) &1;\
      uint32_t imm32 = ((imm11 << 1) | (imm10 << 12) | (I2 << 22) | (I1 << 23)) - (1<<24);\
      do_exec({\
        uint32_t addr = m_nextPC + imm32;\
        do_trace({\
          std::cout << "BL " << std::hex << addr << std::dec << std::endl;\
        })\
        LR() = m_nextPC;\
        m_nextPC = addr;\
      })\
    }\
  }

#define OPCODE_1111_00x_bl(opcode, do_disasm, do_exec, do_trace, core) \
  { \
    if ((opcode & 0x0000'5000) == 0x0000'0000) {\
      OPCODE_1111_001_mov_spec(opcode, do_disasm, do_exec, do_trace, core)\
    } else {\
      uint8_t cond = (opcode >> 24) & 0xf; \
      uint32_t imm11 = (opcode) & 0x7ff; \
      uint32_t imm10 = (opcode >> 16) & 0x3ff; \
      uint32_t I1 = 1^((opcode >> 13) &1);\
      uint32_t I2 = 1^((opcode >> 11) &1);\
      uint32_t imm32 = ((imm11 << 1) | (imm10 << 12) | (I2 << 22) | (I1 << 23));\
      do_exec({\
        uint32_t addr = m_nextPC + imm32;\
        do_trace({\
          std::cout << "BL " << std::hex << addr << std::dec << std::endl;\
        })\
        LR() = m_nextPC;\
        m_nextPC = addr;\
      })\
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
  /*0001_000*/ o(0b0001'000, OPCODE_0001_0xx_asr) \
  /*0001_001*/ o(0b0001'001, OPCODE_0001_0xx_asr) \
  /*0001_010*/ o(0b0001'010, OPCODE_0001_0xx_asr) \
  /*0001_011*/ o(0b0001'011, OPCODE_0001_0xx_asr) \
  /*0001_100*/ o(0b0001'100, OPCODE_0001_100_add) \
  /*0001_101*/ o(0b0001'101, OPCODE_0001_101_sub) \
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
  /*0100_001*/ o(0b0100'001, OPCODE_0100_00x_arith) \
  /*0100_010*/ o(0b0100'010, OPCODE_0100_010_special) \
  /*0100_001*/ o(0b0100'011, OPCODE_0100_011_mov) \
  /*0100_100*/ o(0b0100'100, OPCODE_0100_1xx_load) \
  /*0100_101*/ o(0b0100'101, OPCODE_0100_1xx_load) \
  /*0100_110*/ o(0b0100'110, OPCODE_0100_1xx_load) \
  /*0100_111*/ o(0b0100'111, OPCODE_0100_1xx_load) \
  /*0101_000*/ o(0b0101'000, OPCODE_0101_000_store) \
  /*0101_001*/ o(0b0101'001, OPCODE_0101_001_strh) \
  /*0101_010*/ o(0b0101'010, OPCODE_0101_010_strb) \
  /*0101_011*/ o(0b0101'011, OPCODE_0101_011_ldrsb) \
  /*0101_100*/ o(0b0101'100, OPCODE_0101_100_ldr) \
  /*0101_101*/ o(0b0101'101, OPCODE_0101_101_ldrh) \
  /*0101_110*/ o(0b0101'110, OPCODE_0101_110_ldrb) \
  /*0101_111*/ o(0b0101'111, OPCODE_0101_111_ldrsh) \
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
  /*1011_011*/ o(0b1011'011, OPCODE_1011_011_pcs) \
  /*1011_100*/ o(0b1011'100, OPCODE_UNDEFINED) \
  /*1011_101*/ o(0b1011'101, OPCODE_1011_101_rev) \
  /*1011_110*/ o(0b1011'110, OPCODE_1011_110_popm) \
  /*1011_111*/ o(0b1011'111, OPCODE_1011_111_misc) \
  /*1100_000*/ o(0b1100'000, OPCODE_1100_0xx_stm) \
  /*1100_001*/ o(0b1100'001, OPCODE_1100_0xx_stm) \
  /*1100_010*/ o(0b1100'010, OPCODE_1100_0xx_stm) \
  /*1100_011*/ o(0b1100'011, OPCODE_1100_0xx_stm) \
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
