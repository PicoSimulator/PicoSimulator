#pragma once

namespace RP2040{
  // class SSI;
  // class SSIFiFoAux;
  class RP2040;
}

// #include "rp2040.hpp"
#include "rp2040/peripheral.hpp"
#include "clock.hpp"
#include "fifo.hpp"

#include "ext/io/spidev.hpp"
#include "ext/chips/w25q_flash.hpp"

#include <array>

namespace RP2040{

  class SSI final: public IPeripheralPort, public IClockable {
  public:
    SSI(RP2040 &rp2040) 
    : m_rp2040{rp2040}
    {}
    virtual ~SSI() {};
    virtual void tick() override;
    W25QFlash &spidev() { return m_spidev; }
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) final override;
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) final override;
    virtual uint32_t read_word_internal_pure(uint32_t addr) const final override;
  private:
    friend class SSIFiFoAux;
    RP2040 &m_rp2040;
    uint32_t m_ctrlr0;
    uint32_t m_ctrlr1;
    uint32_t m_ssienr;
    uint32_t m_mwcr;
    uint32_t m_ser;
    uint32_t m_baudr;
    uint32_t m_txftlr;
    uint32_t m_rxftlr;
    FiFo<uint32_t, 16> m_tx_fifo;
    FiFo<uint32_t, 16> m_rx_fifo;
    uint32_t m_sr;
    uint32_t m_imr;
    uint32_t m_isr;
    uint32_t m_risr;
    uint32_t m_txoicr;
    uint32_t m_rxuicr;
    uint32_t m_spi_ctrlr0;


    W25QFlash m_spidev;

    enum RegOffset{
      CTRLR0 = 0x00,
      CTRLR1 = 0x04,
      SSIENR = 0x08,
      MWCR = 0x0c,
      SER = 0x10,
      BAUDR = 0x14,
      TXFTLR = 0x18,
      RXFTLR = 0x1c,
      TXFLR = 0x20,
      RXFLR = 0x24,
      SR = 0x28,
      IMR = 0x2c,
      ISR = 0x30,
      RISR = 0x34,
      TXOICR = 0x38,
      RXOICR = 0x3c,
      RXUICR = 0x40,
      MSTICR = 0x44,
      ICR = 0x48,
      DMACR = 0x4c,
      DMATDLR = 0x50,
      DMARDLR = 0x54,
      IDR = 0x58,
      SSI_VERSION_ID = 0x5c,
      DR0 = 0x60,
      RX_SAMPLE_DLY = 0xf0,
      SPI_CTRLR0 = 0xf4,
      TXD_DRIVE_EDGE = 0xf8,
    };
  };

}