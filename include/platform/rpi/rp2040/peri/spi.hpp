#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "common/interrupt.hpp"
#include "platform/rpi/rp2040/pad.hpp"

namespace RP2040 {
  class RP2040;
}

class SPI final/*?*/ : public IPeripheralPort {
public:
  enum Register {
    SSPCR0 = 0x0,
    SSPCR1 = 0x4,
    SSPDR = 0x8,
    SSPSR = 0xC,
    SSPCPSR = 0x10,
    SSPIMSC = 0x14,
    SSPRIS = 0x18,
    SSPMIS = 0x1C,
    SSPICR = 0x20,
    SSPDMACR = 0x24,
    SSPPERIPHID0 = 0xFE0,
    SSPPERIPHID1 = 0xFE4,
    SSPPERIPHID2 = 0xFE8,
    SSPPERIPHID3 = 0xFEC,
    SSPPCELLID0 = 0xFF0,
    SSPPCELLID1 = 0xFF4,
    SSPPCELLID2 = 0xFF8,
    SSPPCELLID3 = 0xFFC
  };
  SPI(InterruptSource &irq_source) 
  : m_irqs(irq_source) 
  , m_PCLK(*this)
  , m_SSPCLK(*this)
  {
    m_mosi.set_oe(true);
    m_sck.set_oe(true);
    m_miso.set_oe(false);
  }
  RP2040::DMA::DReqSource &tx_dreq() { return m_tx_fifo.dreq(); }
  RP2040::DMA::DReqSource &rx_dreq() { return m_rx_fifo.dreq(); }

  IClockable &PCLK() { return m_PCLK; }
  IClockable &SSPCLK() { return m_SSPCLK; }

  GPIOSignal &TX() { return m_mosi; }
  GPIOSignal &RX() { return m_miso; }
  GPIOSignal &SCK() { return m_sck; }
  GPIOSignal &CSn() { return m_csn; }


protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    // std::cerr << "SPI::read_word_internal(0x" << std::hex << addr << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    case SSPCR0:
      out = m_SSPCR0;
      break;
    case SSPCR1:
      out = m_SSPCR1;
      break;
    case SSPDR:
      out = m_rx_fifo.pop();
      break;
    case SSPSR: out = sspsr(); break;
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    // std::cerr << "SPI::write_word_internal(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    case SSPCR0:
      m_SSPCR0 = in;
      break;
    case SSPCR1:
      m_SSPCR1 = in;
      break;
    case SSPDR:
      m_tx_fifo.push(in);
      break;
    case SSPSR:
      // m_SSPSR = in;
      break;
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {return 0;}

  uint8_t numbits() const { return dss()+1; }
  uint8_t dss() const { return (m_SSPCR0 >> 0) & 0xf; }
  uint8_t frf() const { return (m_SSPCR0 >> 4) & 0x3; }
  uint8_t cpol() const { return (m_SSPCR0 >> 6) & 0x1; }
  uint8_t cpha() const { return (m_SSPCR0 >> 7) & 0x1; }
  uint8_t scr() const { return (m_SSPCR0 >> 8) & 0xff; }

  uint8_t prescale_divisor() const { return m_SSPCPSR & 0xfe; }

  bool loopback() const { return m_SSPCR1 & 0x1; }
  bool enabled() const { return m_SSPCR1 & 0x2; }
  bool master() const { return !(m_SSPCR1 & 0x4); }
  bool slave() const { return m_SSPCR1 & 0x4; }

  bool busy() const { return m_shiftBits != 0 || !m_tx_fifo.empty() || m_sck.get_output() != m_cpol; }

  InterruptSource &rorintr() { return m_irqs[0]; }
  InterruptSource &rtintr() { return m_irqs[1]; }
  InterruptSource &rxintr() { return m_irqs[2]; }
  InterruptSource &txintr() { return m_irqs[3]; }

  uint32_t sspsr() const
  {
    return 
      (m_tx_fifo.empty() << 0) |
      (!m_tx_fifo.full() << 1) |
      (!m_rx_fifo.empty() << 2) |
      (m_rx_fifo.full() << 3) |
      (busy() << 4); 
  }
private:

  void sspclkdiv_tick()
  {
    bool &cpol = m_cpol;
    if (m_shiftBits) {
      bool current_clk = m_sck.get_output();
      if (current_clk == cpol && !data_ready) {
        // idle low, prepare data
        bool bit_out = (m_shiftDataOut >> (m_shiftBits)) & 1;
        m_mosi.set_output(bit_out);
        data_ready = true;
      } else if (current_clk == cpol && data_ready) {
        // data ready, clock data in/out
        bool bit_in = m_miso.get_output();
        m_sck.set_output(!cpol);  
        m_shiftBits--;
        data_ready = false;
        // m_shiftDataOut >>= 1;
        m_shiftDataIn |= bit_in << m_shiftBits;
        if (m_shiftBits == 0) {
          m_rx_fifo.push(m_shiftDataIn);
          if (m_rx_fifo.full()) rorintr().raise();
          if (m_rx_fifo.empty()) rtintr().raise();
        }
      } else {
        // transition to idle, prepare data
        bool bit_out =(m_shiftDataOut >> (m_shiftBits)) & 1;
        m_mosi.set_output(bit_out);
        m_sck.set_output(cpol);
        data_ready = true;
        // std::cerr << "SPI::sspclkdiv_tick" << std::endl;
      }
    } else {
      // idle clock
      m_sck.set_output(cpol);
      if (!m_tx_fifo.empty()) {
        m_shiftDataOut = m_tx_fifo.pop() << 1;
        m_shiftDataIn = 0;
        m_shiftBits = numbits();
        
        bool bit_out = (m_shiftDataOut >> (m_shiftBits)) & 1;
        m_mosi.set_output(bit_out);
        m_sck.set_output(cpol);
        data_ready = true;
      }
    }
  }

  class SPI_PCLK final : public IClockable {
  public:
    SPI_PCLK(SPI &spi) : m_spi{spi} {}
  private:
    virtual void tick() override final
    {
      // m_spi.sspclkdiv_tick();
    }
    SPI &m_spi;
  } m_PCLK;
  class SPI_SSPCLK final : public IClockable {
  public:
    SPI_SSPCLK(SPI &spi) : m_spi{spi} {}
  private:
    virtual void tick() override final
    {
      if (m_div-- != 0) return;
      m_div = m_spi.prescale_divisor()-1;
      if (m_div2-- != 0) return;
      m_div2 = m_spi.scr();
      m_spi.sspclkdiv_tick();
    }
    SPI &m_spi;
    uint8_t m_div;
    uint8_t m_div2;
  } m_SSPCLK;
  InterruptSourceMulti m_irqs;
  uint32_t m_SSPCR0;
  uint32_t m_SSPCR1;
  uint8_t m_SSPCPSR;
  DReqTxFiFo<uint16_t, 8> m_tx_fifo;
  DReqRxFiFo<uint16_t, 8> m_rx_fifo;
  uint8_t m_shiftBits;
  uint32_t m_shiftDataOut;
  uint32_t m_shiftDataIn;
  GPIOSignal m_sck;
  GPIOSignal m_miso;
  GPIOSignal m_mosi;
  GPIOSignal m_csn;
  SPIDev *m_dev;
  bool data_ready = false;
  bool m_cpol = 0;
};