#pragma once

#include "memory_device.hpp"
#include "clock.hpp"
#include "reset.hpp"
#include "async.hpp"
#include "bus.hpp"
#include "nvic.hpp"
#include "armv6m/exception.hpp"

#include <coroutine>
#include <utility>
#include <string>
#include <tuple>
#include <span>

namespace ARMv6M{

  class ARMv6MCore final : public IClockable, public IResettable{
  public:
    ARMv6MCore(IAsyncReadWritePort<uint32_t> &bus, std::string name, std::span<ARMv6MCore> all_cores, InterruptSourceSet &irqs) 
    : m_mpu_bus_interface{bus, *this}
    , m_ppb{*this}
    , m_core_task{core_task()}
    , m_nvic{m_core_irqs, irqs}
    , m_name{std::move(name)}
    , m_cores{all_cores}
    {

    }
    void tick() override;
    virtual void reset() override;
    BusMaster *run() { return &m_core_task; }
    const uint32_t &VTOR() const { return m_VTOR; }
    uint32_t &VTOR() { return m_VTOR; }
    NVIC &nvic() { return m_nvic; }
  protected:
  private:
    int instruction_count = 0;

    void set_PC(uint32_t addr) { m_regs[15] = addr; }
    uint32_t &PC() { return m_regs[15]; }
    uint32_t &LR() { return m_regs[14]; }
    uint32_t &SP() { return m_regs[13]; }
    const uint32_t &PC() const { return m_regs[15]; }
    const uint32_t &LR() const { return m_regs[14]; }
    const uint32_t &SP() const { return m_regs[13]; }
    uint32_t XPSR() const { return m_APSR | m_IPSR | m_EPSR; }
    void set_reg(int reg, uint32_t val) { m_regs[reg] = val; }
    uint32_t get_reg(int reg) const { return m_regs[reg]; }
    bool T() const { return m_regs[15] & 1; }

    uint32_t instr_addr() const { return PC() & ~1; }

    Awaitable<void> exec_instr(uint32_t instr);

    
    static uint32_t SignExtend(uint32_t v, int bits) { 
      return v | (- (v & (1 << (bits - 1))));
    }
    static std::tuple<uint32_t, bool, bool> AddWithCarry(uint32_t a, uint32_t b, bool carry_in)
    {
      uint64_t result = (uint64_t)a + (uint64_t)b + (uint64_t)carry_in;
      bool carry_out = result & 0x100000000;
      bool overflow = ((a & 0x80000000) == (b & 0x80000000)) && ((a & 0x80000000) != (result & 0x80000000));
      return {result, carry_out, overflow};
    }

    bool CurrentModeIsPrivileged() const { return !m_threadMode || !(m_CONTROL & CONTROL::nPRIV); }
    void set_MSP(uint32_t val) { m_MSP = val; SP() = val; }
    void set_PSP(uint32_t val) { m_PSP = val; }
    // TODO: Interworking!
    void BLXWritePC(uint32_t target) { m_nextPC = target; }
    void BXWritePC(uint32_t target) { 
      if ((target & 0xf000'0000) == 0xf000'0000) {
        std::cout << "BXWritePC: target = " << std::hex << target << std::dec << "\n";
        // std::terminate();
        // throw ARMv6M::HardFault{"Interworking exception returtn!"};
        m_interworking_required = true;
      }
      m_nextPC = target; 
    }
    void BranchTo(uint32_t target) { m_nextPC = target | PC() & 1; }
    void BranchWritePC(uint32_t target) { BranchTo(target&~1); }
    void ALUWritePC(uint32_t target) { BranchWritePC(target); }
    void LoadWritePC(uint32_t target) { BXWritePC(target); }
    bool m_event;
    bool EventRegistered()  const { return m_event; }
    void ClearEventRegister() { m_event = false; }
    void SendEvent() { for (ARMv6MCore &core : m_cores) core.m_event = true; }

    bool ExceptionActive(int num) {
      return m_active_exceptions[num];
    }
    NVIC::exception_bits m_active_exceptions;
    uint8_t m_current_exception;

    // enum ProcessorMode{
    //   ThreadMode = 0b10000,
    //   HandlerMode = 0b11011,
    //   ModeMask = 0b11111,
    // };

    InterruptSourceSet m_core_irqs;


    uint64_t m_tickcnt = 0;
    uint32_t m_regs[16];
    uint32_t m_nextPC;
    uint32_t m_MSP, m_PSP;
    uint32_t m_APSR;
    uint32_t m_IPSR;
    uint32_t m_EPSR;
    uint32_t m_PRIMASK;
    uint32_t m_CONTROL;
    uint32_t m_VTOR;
    enum CONTROL{
      nPRIV = 1 << 0,
      SPSEL = 1 << 1,
    };
    enum APSR{
      OVERFLOW = 1<<28,
      CARRY = 1<<29,
      ZERO = 1<<30,
      NEGATIVE = 1<<31,
    };
    bool m_threadMode;
    bool m_interworking_required = false;

    enum Cond{
      EQ = 0b0000,
      NE = 0b0001,
      CS = 0b0010,
      CC = 0b0011,
      MI = 0b0100,
      PL = 0b0101,
      VS = 0b0110,
      VC = 0b0111,
      HI = 0b1000,
      LS = 0b1001,
      GE = 0b1010,
      LT = 0b1011,
      GT = 0b1100,
      LE = 0b1101,
    };

    enum class ExceptionType{
      Reset = 1,
      NMI = 2,
      HardFault = 3,
      SVC = 11,
      PendSV = 14,
      SysTick = 15,
    };
    uint32_t ReturnAddress(ExceptionType type){
      switch(type){
        case ExceptionType::Reset: return 0;
        case ExceptionType::NMI: return 0;
        case ExceptionType::HardFault: return 0;
        case ExceptionType::SVC: return 0;
        case ExceptionType::PendSV: return 0;
        case ExceptionType::SysTick: return 0;
      }
    }

    auto next_tick(){
      m_waiting_for_tick = true;
      return std::suspend_always{};
    }
    void dump() const;
    BusMaster core_task();
    BusMaster m_core_task;
    bool m_waiting_for_tick = true;
    bool m_in_reset = false;

    class MPU final : public IAsyncReadWritePort<uint32_t>{
    public:
      MPU(IAsyncReadWritePort<uint32_t> &bus, ARMv6MCore &core);
    protected:
      virtual bool register_op(MemoryOperation &op) override final{
        assert(m_op == nullptr);
        m_op = &op;
        if(m_waiting_on_op)
          m_runner.resume();
        return m_waiting_on_op;
      }
    private:
      MemoryOperation *m_op;
      std::coroutine_handle<> m_runner;
      bool m_waiting_on_op;
      auto next_op(){
        struct awaitable{
          bool await_ready() {
            return false;
            return m_bus.m_op != nullptr; 
          }
          MemoryOperation &await_resume() { 
            m_bus.m_waiting_on_op = false;
            return *m_bus.m_op; 
          }
          void await_suspend(std::coroutine_handle<> handle) {
            m_bus.m_waiting_on_op = true;
            if (m_bus.m_op != nullptr) {
              auto *op = m_bus.m_op;
              m_bus.m_op = nullptr;
              op->complete();
            }
          }
          MPU &m_bus;
        };
        // if (m_op != nullptr) {
        //   auto *op = m_op;
        //   m_op = nullptr;
        //   op->complete();
        // }
        return awaitable{*this};
      }
      Task bus_task();
    private:
      IAsyncReadWritePort<uint32_t> &m_bus_interface;
      ARMv6MCore &m_core;
    };

    class PPB final : public IReadWritePort<uint32_t>{
    public:
      PPB(ARMv6MCore &core) : m_core{core} {}
      virtual PortState read_byte(uint32_t addr, uint8_t &out) override;
      virtual PortState read_halfword(uint32_t addr, uint16_t &out) override;
      virtual PortState read_word(uint32_t addr, uint32_t &out) override;
      virtual PortState write_byte(uint32_t addr, uint8_t in) override;
      virtual PortState write_halfword(uint32_t addr, uint16_t in) override;
      virtual PortState write_word(uint32_t addr, uint32_t in) override;
    protected:
    private:
      ARMv6MCore &m_core;
    };

    MPU m_mpu_bus_interface;
    PPB m_ppb;
    IAsyncReadWritePort<uint32_t> &m_bus_interface = m_mpu_bus_interface;
    NVIC m_nvic;
    const std::string m_name;
    std::span<ARMv6MCore> m_cores;
  };


}
