#include "rp2040/peri/ssi.hpp"

using namespace RP2040;

void SSI::tick()
{
  if ((m_ctrlr0 & 0x000f'f000) == 0x0007'0000 && m_tx_fifo.count() > 0 && !m_rx_fifo.full()) {
    if (m_ctrlr0 & 0x0080'0000) {
      spidev().set_cs(0);
    }
    switch(m_ctrlr0 & 0x0030'0000) {
      case 0x0000'0000: /*Standard SPI, 1-bit per SCK, full-duplex*/
      case 0x0010'0000: /*Dual SPI, 2-bit per SCK, half-duplex*/
      case 0x0020'0000: /*QUAD SPI, 4-bit per SCK, half-duplex*/;
    }
    m_rx_fifo.push(spidev().spi_exchange_byte(m_tx_fifo.pop()));
    if (m_ctrlr0 & 0x0080'0000) {
      spidev().set_cs(1);
    }
  }
}
 
PortState SSI::read_word_internal(uint32_t addr, uint32_t &out)
{
  switch(addr & 0xfc) {
    case RegOffset::CTRLR0: out = m_ctrlr0; break;
    case RegOffset::CTRLR1: out = m_ctrlr1; break;
    case RegOffset::TXFLR: out = m_tx_fifo.count(); break;
    case RegOffset::RXFLR: out = m_rx_fifo.count(); break;
    case RegOffset::SR: out = 0x0000'0004; break;
    case RegOffset::DR0: out = m_rx_fifo.pop(); break;
    case RegOffset::SPI_CTRLR0: out = m_spi_ctrlr0; break;
    default: return PortState::FAULT;
  }
  return PortState::SUCCESS;
}

PortState SSI::write_word_internal(uint32_t addr, uint32_t in)
{
  switch(addr & 0xfc) {
    case RegOffset::CTRLR0: m_ctrlr0 = in; break;
    case RegOffset::CTRLR1: m_ctrlr1 = in; break;
    case RegOffset::DR0: m_tx_fifo.push(in); break;
    case RegOffset::SPI_CTRLR0: m_spi_ctrlr0 = in; break;
    default: return PortState::FAULT;
  }
  return PortState::SUCCESS;
}
uint32_t SSI::read_word_internal_pure(uint32_t addr) const
{
  return 0;
}