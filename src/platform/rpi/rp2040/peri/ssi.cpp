#include "platform/rpi/rp2040/peri/ssi.hpp"
#include <bitset>

using namespace RP2040;

void SSI::tick()
{
  if (/* (m_ctrlr0 & 0x000f'f000) == 0x0007'0000 && */ m_tx_fifo.count() > 0 && !m_rx_fifo.full()) {
    if (m_ctrlr0 & 0x0080'0000 && spidev()) {
      spidev()->set_cs(0);
    }
    switch(m_ctrlr0 & 0x0030'0000) {
      case 0x0000'0000: /*Standard SPI, 1-bit per SCK, full-duplex*/
      case 0x0010'0000: /*Dual SPI, 2-bit per SCK, half-duplex*/
      case 0x0020'0000: /*QUAD SPI, 4-bit per SCK, half-duplex*/;
    }
    std::cout << "txpop[" << std::dec << m_tx_fifo.count() << "]" << std::endl;
    std::cout << "rxpush[" << std::dec << m_rx_fifo.count() << "]" << std::endl;
    uint32_t rx = 0, tx = m_tx_fifo.pop();
    if (spidev()) {
      rx = spidev()->spi_exchange_byte(tx);
    }
    m_rx_fifo.push(rx);
    if (m_ctrlr0 & 0x0080'0000 && spidev()) {
      spidev()->set_cs(1);
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
    case RegOffset::SR: 
      out = (m_tx_fifo.full() ? 0x0000'0000 : 0x0000'0002)
          | (m_tx_fifo.empty() ? 0x0000'0004 : 0x0000'0000)
          | (m_rx_fifo.empty() ? 0x0000'0000: 0x0000'0008)
          | (m_rx_fifo.full() ? 0x0000'0010 : 0x0000'0000); 
      // out = 0x0000'0004;
      break;
    case RegOffset::DR0: 
      std::cout << m_rx_fifo.count() << std::endl;
      out = m_rx_fifo.pop(); 
      std::cout << "rxpop[" << std::dec << m_rx_fifo.count() << "]" << std::endl;
      break;
    case RegOffset::SPI_CTRLR0: out = m_spi_ctrlr0; break;
    default: return PortState::FAULT;
  }
  std::cout << "SSI::read_word_internal(" << std::hex << addr << ") -> " << std::bitset<32>{out} << std::endl;
  return PortState::SUCCESS;
}

PortState SSI::write_word_internal(uint32_t addr, uint32_t in)
{
  std::cout << "SSI::write_word_internal(" << std::hex << addr << ", " << in << ")" << std::endl;
  switch(addr & 0xfc) {
    case RegOffset::CTRLR0: m_ctrlr0 = in; break;
    case RegOffset::CTRLR1: m_ctrlr1 = in; break;
    case RegOffset::SSIENR: {
      m_ssienr = in; 
      if ((m_ssienr & 0x0000'0001) == 0) {
        m_tx_fifo.clear();
        m_rx_fifo.clear();
      }
    }break;
    case RegOffset::DR0: if(!m_tx_fifo.full()){
      std::cout << "txpush[" << std::hex << m_tx_fifo.count() << "]" << std::endl;
      m_tx_fifo.push(in); 
    }break;
    case RegOffset::SPI_CTRLR0: m_spi_ctrlr0 = in; break;
    default: return PortState::FAULT;
  }
  return PortState::SUCCESS;
}
uint32_t SSI::read_word_internal_pure(uint32_t addr) const
{
  return 0;
}