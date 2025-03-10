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
    uint8_t exec_instr(uint16_t instr);
    void tick();
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