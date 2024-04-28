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
    , m_spidev{new W25QFlash()} 
    {}
    virtual ~SSI() {};
    virtual void tick() override;
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) final override;
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState xor_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState set_bits_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState clear_bits_word_internal(uint32_t addr, uint32_t in) final override;
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


    SPIDev *m_spidev;
  };

  class SSIFiFoAux : public IPeripheralPort{
  public:
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) final override;
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState xor_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState set_bits_word_internal(uint32_t addr, uint32_t in) final override;
    virtual PortState clear_bits_word_internal(uint32_t addr, uint32_t in) final override;
  private:
    SSI &m_ssi;
  };


}