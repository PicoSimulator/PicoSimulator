#pragma once

#include "clock.hpp"

namespace RP2040{
  using PIOInstrMemory = std::array<uint16_t, 32>;
  class PIO final : public IClockable{
  public:
    PIO();
    void tick();
  protected:
  private:
  };
  class PIOStateMachine final : public IClockable{
  public:
    PIOStateMachine(PIOBlock &block, uint32_t id)
    : m_block{block}
    , m_id{id}
    {}
    void tick();
  protected:
  private:
    uint32_t m_scratch_x, m_scratch_y;
    uint32_t m_osr, m_isr;
    uint32_t m_pc;
    PIOBlock &m_block;
    uint32_t m_id;
  };
  class PIOBlock final : public IClockable{
  public:
    const PIOInstrMemory &instrmem() const { return m_instrmem; }
  protected:
  private:
    PIOStateMachine m_statemachines[4];
    PIOInstrMemory m_instrmem;
  };
}