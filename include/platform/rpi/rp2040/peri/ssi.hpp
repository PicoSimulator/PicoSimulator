#pragma once

#include "platform/rpi/rp2040/gpio.hpp"

namespace RP2040{
  // class SSI;
  // class SSIFiFoAux;
  class RP2040;
}

// #include "platform/rpi/rp2040/rp2040.hpp"
#include "platform/rpi/rp2040/peripheral.hpp"
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
    {
      m_ss.set_oe(true);
      m_ss.set_output(true);
    }
    virtual ~SSI() {};
    virtual void tick() override;
    W25QFlash *spidev() { return m_spidev; }
    std::span<uint8_t> flash() const { return m_flash; }
    void set_spidev(W25QFlash *spidev) { 
      m_spidev = spidev; 
      if (!m_spidev) {
        m_flash = m_flash_priv;
      } else {
        m_flash = m_spidev->flash();
      }

    }
    GPIOSignal &SCK() { return m_sck; }
    GPIOSignal &SS() { return m_ss; }
    GPIOSignal &D0() { return m_d0; }
    GPIOSignal &D1() { return m_d1; }
    GPIOSignal &D2() { return m_d2; }
    GPIOSignal &D3() { return m_d3; }
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


    W25QFlash *m_spidev;

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
  
    GPIOSignal m_sck, m_ss, m_d0, m_d1, m_d2, m_d3;
    std::array<uint8_t, 0x0100'0000> m_flash_priv;
    std::span<uint8_t> m_flash;
  };

}