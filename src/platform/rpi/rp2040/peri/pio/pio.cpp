#include "platform/rpi/rp2040/peri/pio/pio.hpp"

using namespace RP2040::PIO;

uint8_t PIOStateMachine::exec_instr(uint16_t instr)
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
            break;
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

void PIOStateMachine::tick()
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
