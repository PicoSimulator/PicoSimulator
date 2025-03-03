#pragma once

#include "clock.hpp"
#include "platform/rpi/rp2040/gpio.hpp"
#include "fifo.hpp"

namespace RP2040{
  using PIOInstrMemory = std::array<uint16_t, 32>;
  class PIO final : public IClockable{
  public:
    PIO();
    void tick() override;
  protected:
  private:
  };

  class PIOClockDivider final : public ClockTransform{
  public:
    PIOClockDivider(PIO &pio, uint32_t mask)
    : m_pio{pio}
    , m_mask{mask}
    {}
    void tick() override;
  class PIOStateMachine final : public IClockable{
  public:
    PIOStateMachine(PIOBlock &block, uint32_t id)
    : m_block{block}
    , m_id{id}
    {}
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
  protected:
  private:
    uint32_t m_scratch_x, m_scratch_y;
    uint32_t m_osr, m_isr;
    uint32_t m_pc;
    PIOBlock &m_block;
    uint32_t m_id;
    uint8_t m_stall;
    bool m_in_shiftdir, m_out_shiftdir;
    bool m_autopull, m_autopush;
    uint8_t m_input_shift_cnt, m_output_shift_cnt;
    uint8_t m_out_base, m_set_base, m_sideset_base, m_in_base;
    uint8_t m_out_count, m_set_count, m_sideset_count;  
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
    }m_tx_fifo;
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
    }m_rx_fifo;
  };

  class PIOBlock final : public IClockable{
  public:
    const PIOInstrMemory &instrmem() const { return m_instrmem; }
    void tick() override;
  protected:
  private:
    void clkdiv_restart(uint32_t mask);
    PIOStateMachine m_statemachines[4];
    PIOInstrMemory m_instrmem;
    GPIOSignal m_gpio[32];
  };
}