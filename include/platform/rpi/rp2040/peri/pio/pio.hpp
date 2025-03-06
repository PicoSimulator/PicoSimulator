#pragma once

#include "clock.hpp"
#include "platform/rpi/rp2040/gpio.hpp"
#include "platform/rpi/rp2040/peripheral.hpp"
#include "fifo.hpp"
#include "util/saturating_counter.hpp"

namespace RP2040::PIO{
  using PIOInstrMemory = std::array<uint16_t, 32>;
  class PIOBlock;
  // class PIO final : public IClockable{
  // public:
  //   PIO();
  //   void tick() override;
  // protected:
  // private:
  // };

  // class PIOClockDivider final : public ClockTransform{
  // public:
  //   PIOClockDivider(PIO &pio, uint32_t mask)
  //   : m_pio{pio}
  //   , m_mask{mask}
  //   {}
  //   void tick() override;
  // };

  class PIOTxFifo final : public DReqTxFiFoBase<uint32_t, 8>{
    public:
      size_t size() const override { return m_size; }
      void set_joined(bool joined, bool this_fifo)
      {
        clear();
        if (!joined) m_size = 4;
        else if (this_fifo) m_size = 8;
        else m_size = 0;
      }
    private:
      std::size_t m_size = 4;
    };
    class PIORxFifo final : public DReqRxFiFoBase<uint32_t, 8>{
      public:
      size_t size() const override { return m_size; }
      void set_joined(bool joined, bool this_fifo)
      {
        clear();
        if (!joined) m_size = 4;
        else if (this_fifo) m_size = 8;
        else m_size = 0;
      }
      private:
      std::size_t m_size = 4;
    };

  class PIOStateMachine final : public IClockable{
  public:
    PIOStateMachine(PIOBlock &block, uint32_t id)
    : m_block{block}
    , m_id{id}
    {}
    uint8_t exec_instr(uint16_t instr)
    {
      uint8_t lo5 =  (instr>>0)&0x1f;
      uint8_t mid3 = (instr>>5)&0x07;
      uint8_t hi5 =  (instr>>8)&0x1f;
      uint16_t nextPC;
      bool was_stalled = m_stall;
      bool was_exec = m_instr_exec;
      if (was_stalled || was_exec) {
        nextPC = m_pc;
      }else if (m_pc == m_wrap_top) {
        nextPC = m_wrap_bottom;
      } else {
        nextPC = (m_pc+1) % 32;
      }
      switch(instr & 0xe000) {
        case 0x0000: // JMP
        {
          bool cond;
          switch(mid3) {
            case 0b000: // always
              cond = true; break;
            case 0b001: // !X
              cond = !m_scratch_x; break;
            case 0b010: // X--
              cond = m_scratch_x--; break;
            case 0b011: // !Y
              cond = !m_scratch_y; break;
            case 0b100: // Y--
              cond = m_scratch_y--; break;
            case 0b101: // X!=Y
              cond = m_scratch_x != m_scratch_y; break;
            case 0b110: // PIN
              cond = m_block.m_gpio[m_jmp_pin].get_input(); break;
            case 0b111: // !OSRE
              cond = !osre(); break;
          }
          if (cond) 
            nextPC = lo5;
        } break;
        case 0x2000: // WAIT
        {
          bool polarity = mid3&4;
          uint8_t src = mid3&3;
          switch(src) {
            case 0b00: // GPIO
              m_stall = m_block.m_gpio[lo5].get_input() != polarity; 
              break;
            case 0b01: // PIN
              m_stall = m_block.m_gpio[(m_in_base + lo5) % 32].get_input() != polarity; 
              break;
            case 0b10: // IRQ
            {
              uint8_t irqNum = irq_decode(lo5);
              auto &irq = m_block.m_irqs[irqNum];
              m_stall = irq.operator bool() != polarity;
              if (polarity)
                irq.lower();
            } break;
            case 0b11: // Reserved
            ;
          }
        } break;
        case 0x4000: // IN
        {
          // convert 0->32, essentially undoing (1..32)%32
          if (!m_stall) {
            uint8_t BitCnt = (lo5-1) & 0x1f + 1;
            uint32_t bits_in = 0;
            switch(mid3) {
              case 0b000: // PINS
              {
                for (int i = 0; i < BitCnt; i++) {
                  if (m_block.m_gpio[(i+m_in_base)%32].get_input()) {
                    bits_in |= 1<<i;
                  }
                }
              } break;
              case 0b001: // X
                bits_in = m_scratch_x & ((1 << BitCnt) - 1);
                break;
              case 0b010: // Y
                bits_in = m_scratch_y & ((1 << BitCnt) - 1);
                break;
              case 0b011: // NULL
              case 0b100: // Reserved
              case 0b101: // Reserved
                break;
              case 0b110: // ISR
              bits_in = m_isr & ((1 << BitCnt) - 1);
              break;
              case 0b111: // OSR
              bits_in = m_osr & ((1 << BitCnt) - 1);
              break;
            }
            m_input_shift_cnt += BitCnt;
            if (m_in_shiftdir) {
              //shift into MSB
              m_isr >>= BitCnt;
              m_isr |= (bits_in << (32-BitCnt));
            } else {
              m_isr <<= BitCnt;
              m_isr |= bits_in;
            }
          }
          // handle autopush
          if (autopush() 
              && m_input_shift_cnt >= m_push_threshold ) {
            if (m_rx_fifo.full()) {
              m_stall = true;
            } else {
              m_rx_fifo.push(m_isr);
              m_input_shift_cnt = 0;
              m_isr = 0;
              m_stall = false;
            }
          }
        } break;
        case 0x6000: // OUT
        {
          if (!m_stall) {
            uint8_t BitCnt = (lo5-1) & 0x1f + 1;
            uint32_t bits_out;
            if (m_out_shiftdir) {
              bits_out = m_osr & ((1<<BitCnt)-1);
              m_osr >>= BitCnt;
            } else {
              bits_out = m_osr >> (32-BitCnt);
              m_osr <<= BitCnt;
            }
            m_output_shift_cnt += BitCnt;
            switch (mid3) {
              case 0b000: // PINS
              {
                for (int i = 0; i < m_out_count; i++) {
                  m_block.m_gpio[(i+m_out_base)%32].set_output(bits_out & 1);
                  bits_out >>= 1;
                }
              } break;
              case 0b001: // X
                m_scratch_x = bits_out; break;
              case 0b010: // Y
                m_scratch_y = bits_out; break;
              case 0b011: // NULL
                break;
              case 0b100: // PINDIRS
              {
                for (int i = 0; i < m_out_count; i++) {
                  m_block.m_gpio[(i+m_out_base)%32].set_oe(bits_out & 1);
                  bits_out >>= 1;
                }
              } break;
              case 0b101: // PC
                nextPC = bits_out; break;
              case 0b110: // ISR
                m_isr = bits_out; 
                m_input_shift_cnt = BitCnt;
                
              case 0b111: // EXEC
                m_exec_instr = bits_out;
                m_instr_exec = true;
                break;
            }
          }
          // handle autopull
          if (autopull() 
              && m_output_shift_cnt >= m_pull_threshold ) {
            if (m_tx_fifo.empty()) {
              m_stall = true;
            } else {
              m_stall = false;
              m_osr = m_tx_fifo.pop();
              m_output_shift_cnt = 0;
            }
          }
        } break;
        case 0x8000: // PUSH/PULL
        {
          bool cond1 = (mid3&4)?(m_output_shift_cnt < m_pull_threshold):(m_input_shift_cnt < m_push_threshold);
          bool cond2 = (mid3&4)?m_tx_fifo.empty():m_rx_fifo.full();
          if (mid3 & 2 && cond1) {
            // skip
          } else if (mid3 & 1 && cond2) {
            // block
            stall = true;
          } else if (mid3 & 4) {
            // pull
            m_osr = m_tx_fifo.pop();
            m_output_shift_cnt = 0;
            m_stall = false;
          } else {
            // push
            m_rx_fifo.push(m_isr);
            m_isr = 0;
            m_input_shift_cnt = 0;
            m_stall = false;
          }
        } break;
        case 0xa000: // MOV
        {
          uint8_t src = lo5 & 7;
          uint8_t dst = mid3;
          uint8_t op = lo5 >> 3;
          uint32_t data;
          switch (src) {
            case 0b000: // PINS
            {
              for (int i = 0; i < 32; i++) {
                data |= m_block.m_gpio[(i+m_in_base)%32].get_input() << i;
              }
            } break;
            case 0b001: // X
              data = m_scratch_x;
              break;
            case 0b010: // Y
              data = m_scratch_y;
              break;
            case 0b011: // NULL
              data = 0;
              break;
            case 0b100: // Reserved
              break;
            case 0b101: // STATUS
              data = status();
              break;
            case 0b110: // ISR
              data = m_isr;
              break;
            case 0b111: // OSR
              data = m_osr;
              break;
          }
          switch (op) {
            case 0b00: // NOOP
              break;
            case 0b01: // INVERT
              data = ~data;
              break;
            case 0b10: // BIT-REV
            {
              uint32_t mask = 0xffffffff;
              // fast bitrev algo
              // O(log(n)) execution time vs
              // naive single bitshift implementation which takes O(n) time
              // this does assume that shifting is constant time independant of 
              // shift count
              for (uint8_t i = 16; i; i>>=1) {
                mask ^= mask >> i;
                data = ((data & mask) >> i) | ((data & ~mask) << i);
              }
            } break;
            case 0b11: // Reserved
          }
          switch (dst) {
            case 0b000: // PINS
            {
              for (int i = 0; i < m_out_count; i++) {
                m_block.m_gpio[(i+m_out_base)%32].set_output(data & i);
                data >>= 1;
              }
            } break;
            case 0b001: // X
              m_scratch_x = data;
              break;
            case 0b010: // Y
              m_scratch_y = data;
              break;
            case 0b011: // Reserved
              break;
            case 0b100: // EXEC
              m_exec_instr = data;
              m_instr_exec = true;
              break;
            case 0b101: // PC
              nextPC = data;
              break;
            case 0b110: // ISR
              m_isr = data;
              m_input_shift_cnt = 0;
              break;
            case 0b111: // OSR
              m_osr = data;
              m_output_shift_cnt = 0;
              break;
          }
        } break;
        case 0xc000: // IRQ
        {
          uint8_t irqNum = irq_decode(lo5);
          auto &irq = m_block.m_irqs[irqNum];
          switch (mid3) {
            case 0b000: // SET, NOWAIT
              irq.raise();
              break;
            case 0b001: // SET, WAIT
              if (!was_stalled) {
                irq.raise();
                m_stall = true;
              }
              if (!irq) {
                m_stall = false;
              }
              break;
            case 0b010: // CLR, NOWAIT
            case 0b011: // CLR, WAIT?
              irq.lower();
              break;
            default:
              break;
          }
        } break;
        case 0xe000: // SET
        {
          switch (mid3) {
            case 0b000: // PINS
            {
              for (int i = 0; i < m_set_count; i++) {
                m_block.m_gpio[(i+m_set_base)%32].set_output(lo5 & 1);
                lo5 >>= 1;
              }
            } break;
            case 0b001: // X
              m_scratch_x = lo5;
              break;
            case 0b010: // Y
              m_scratch_y = lo5;
              break;
            case 0b101: // Reserved
            case 0b011: // Reserved
              break;
            case 0b100: // PINDIRS
            {
              for (int i = 0; i < m_set_count; i++) {
                m_block.m_gpio[(i+m_set_base)%32].set_oe(lo5 & 1);
                lo5 >>= 1;
              }
            } break;
            case 0b101: // Reserved
            case 0b110: // Reserved
            case 0b111: // Reserved
              break;
          }
        } break;
      }
      // handle side-set
      // only if we were un-stalled (ie first execution of instruction only!)
      uint8_t side_set = hi5 & ((1 << m_sideset_count) - 1);
      if (m_side_en && !was_stalled) {
        if (side_set & (1 << (m_sideset_count-1))) {
          for (int i = 0; i < m_sideset_count-1; i++) {
            m_block.m_gpio[(m_sideset_base+i)%32];
          }
        }
      } else if (!was_stalled){
        for (int i = 0; i < m_sideset_count; i++) {
          m_block.m_gpio[(m_sideset_base+i)%32];
        }
      }

      m_pc = nextPC;
      uint8_t delay_bits = 5 - m_sideset_count;
      m_delay_cycles = hi5 >> m_sideset_count;
    }
    void tick()
    {
      uint8_t instr;
      // execute forced/exec'd instructions immediately
      // skips OUT EXEC delay cycles!
      if (m_instr_exec) {
        instr = m_exec_instr;
      // } else if ({
      // if we're stalled or we don't have delay cycles, just execute the instruction
      } else if (!m_stall && m_delay_cycles) {
        m_delay_cycles--;
        return;
      } else {
        if (!m_stall) {
          m_instr = m_block.instrmem()[m_pc];
        }
        instr = m_instr;
      }
      exec_instr(m_instr);
      if (!m_stall) {
        m_instr_exec = false;
      }
    }
    uint32_t pinctrl() const { 
      return 
          m_out_base << 0
        | m_set_base << 5
        | m_sideset_base << 10
        | m_in_base << 15
        | m_out_count << 20
        | m_set_count << 26
        | m_sideset_count << 29;
    }
    void pinctrl(uint32_t v) { 
      m_out_base = v & 0x1f;
      m_set_base = (v >> 5) & 0x1f;
      m_sideset_base = (v >> 10) & 0x1f;
      m_in_base = (v >> 15) & 0x1f;
      m_out_count = (v >> 20) & 0x1f;
      m_set_count = (v >> 26) & 0x7;
      m_sideset_count = (v >> 29) & 0x7;
    }
    uint8_t addr() const { return m_pc; }
    uint32_t shiftctrl() const { return m_isr | (m_osr << 16); }
    const PIOTxFifo &tx_fifo() const { return m_tx_fifo; }
    const PIORxFifo &rx_fifo() const { return m_rx_fifo; }
    bool osre() const { return m_output_shift_cnt >= m_pull_threshold; }
    bool autopush() const { return m_autopush; }
    bool autopull() const { return m_autopull; }
    // auto &gpios() { return m_block.gpio; }
    uint32_t status() const
    {
      uint32_t out;
      if (!m_status_sel) {
        out = m_tx_fifo.count() < m_status_n ? -1 : 0;
      } else {
        out = m_rx_fifo.count() < m_status_n ? -1 : 0;
      }
      return out;
    }
  protected:
    uint8_t irq_decode(uint8_t val) {
      uint8_t irqNum = val & 0x07;
      if (lo5 & 0x10) {
        irqNum = irqNum & 0x04 + (irqNum + m_id) & 3;
      }
      return irqNum;
    }
  private:
    PIOBlock &m_block;
    PIOTxFifo m_tx_fifo;
    PIORxFifo m_rx_fifo;
    uint32_t m_scratch_x, m_scratch_y;
    uint32_t m_osr, m_isr;
    uint16_t m_instr, m_exec_instr;
    uint8_t m_id;
    uint8_t m_pc;
    uint8_t m_wrap_top, m_wrap_bottom;
    uint8_t m_delay_cycles;
    uint8_t m_pull_threshold, m_push_threshold;
    saturating_counter<uint8_t, 32> m_input_shift_cnt, m_output_shift_cnt;
    uint8_t m_out_base, m_set_base, m_sideset_base, m_in_base;
    uint8_t m_out_count, m_set_count, m_sideset_count;  
    uint8_t m_jmp_pin;
    uint8_t m_status_n;
    // 0: shift left, 1: shift right
    bool m_in_shiftdir, m_out_shiftdir;
    bool m_autopull, m_autopush;
    bool m_side_en;
    bool m_stall;
    bool m_instr_exec;
  };

  class PIOBlock final : public IClockable, public IPeripheralPort{
    friend class PIOStateMachine;
  public:
    const PIOInstrMemory &instrmem() const { return m_instrmem; }
    void tick() override;
    uint32_t flevel() const 
    {
      uint32_t out = 0;
      for (int i = 0; i < 4; i++) {
        auto &sm = m_statemachines[i];
        out |= sm.tx_fifo().FiFoBase<uint32_t, 8>::count() << (i * 8 + 0);
        out |= sm.rx_fifo().FiFoBase<uint32_t, 8>::count() << (i * 8 + 4);
      }
      return out;
    }

    uint32_t fstat() const
    {
      uint32_t out = 0;
      for (int i = 0; i < 4; i++) {
        auto &sm = m_statemachines[i];
        out |= sm.tx_fifo().FiFoBase<uint32_t, 8>::full() << (i * 8 + 0);
        out |= sm.rx_fifo().FiFoBase<uint32_t, 8>::full() << (i * 8 + 4);
      }
      return out;
    }
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
    {
      out = 0; 
      switch(addr & 0x1fc) {
        case 0x00: // CTRL
        case 0x04: // FSTAT
        case 0x08: // FDEBUG
        case 0x0c: // FLEVEL
        ;
      }
      return PortState::SUCCESS;
    }
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
    {
      return PortState::SUCCESS;
    }
    virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
    {
      return 0;
    }
  
  private:
    void clkdiv_restart(uint32_t mask);
    PIOStateMachine m_statemachines[4];
    PIOInstrMemory m_instrmem;
    GPIOSignal m_gpio[32];
    InterruptSourceMulti m_irqs;
  };
}