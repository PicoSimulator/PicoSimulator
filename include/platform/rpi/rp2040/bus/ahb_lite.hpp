#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"

#include "platform/rpi/rp2040/peri/dma/dma.hpp"
#include "platform/rpi/rp2040/peri/pio/pio.hpp"

namespace RP2040::Bus {
  class AHBLite final : public IPeripheralPort, public IClockable {
  public:
    AHBLite(DMA::DMA &dma)
    : m_runner{bus_task()}
    , m_dma{dma}
    , m_pio{
      PIO::PIOBlock{},
      PIO::PIOBlock{}
    }
    {}
    virtual void tick() override;
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override ;
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) override ;
    virtual uint32_t read_word_internal_pure(uint32_t addr) const override final ;
  private:
    ClockTask bus_task();
    ClockTask m_runner;

    DMA::DMA &m_dma;
    PIO::PIOBlock m_pio[2];

  };

}