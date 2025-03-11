#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "common/interrupt.hpp"
#include "platform/rpi/rp2040/pad.hpp"
#include "platform/rpi/rp2040/gpio.hpp"

namespace RP2040 {
  class RP2040;
}

class I2C final/*?*/ : public IPeripheralPort {
public:
  I2C(InterruptSource &irq_source) 
  : m_irqs(irq_source) 
  {}
  GPIOSignal &SDA() { return m_sda; }
  GPIOSignal &SCL() { return m_scl; }


protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    std::cerr << "I2C::read_word_internal(0x" << std::hex << addr << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    case 0x00: // IC_CON
    case 0x04: // IC_TAR  
    case 0x08: // IC_SAR  
    case 0x10: // IC_DATA_CMD  
    case 0x14: // IC_SS_SLC_HCNT  
    case 0x18: // IC_SS_SLC_LCNT  
    case 0x1c: // IC_FS_SLC_HCNT  
    case 0x20: // IC_FS_SLC_LCNT  
    case 0x2c: // IC_INTR_STAT  
    case 0x30: // IC_INTR_MASK  
    case 0x34: // IC_RAW_INTR_STAT  
    case 0x38: // IC_RX_TL  
    case 0x3C: // IC_TX_TL  
    case 0x40: // IC_CLR_INTR  
    case 0x44: // IC_CLR_RX_UNDER  
    case 0x48: // IC_CLR_RX_OVER  
    case 0x4C: // IC_CLR_TX_OVER  
    case 0x50: // IC_CLR_RD_REQ  
    case 0x54: // IC_CLR_TX_ABRT  
    case 0x58: // IC_CLR_RX_DONE  
    case 0x5C: // IC_CLR_ACTIVITY  
    case 0x60: // IC_CLR_STOP_DET  
    case 0x64: // IC_CLR_START_DET  
    case 0x68: // IC_CLR_GEN_CALL  
    case 0x6C: // IC_ENABLE
    case 0x70: // IC_STATUS
    case 0x74: // IC_TXFLR
    case 0x78: // IC_RXFLR
    case 0x7C: // IC_SDA_HOLD
    case 0x80: // IC_TX_ABRT_SOURCE
    case 0x84: // IC_SLV_DATA_NACK_ONLY
    case 0x88: // IC_DMA_CR
    case 0x8C: // IC_DMA_TDLR
    case 0x90: // IC_DMA_RDLR
    case 0x94: // IC_SDA_SETUP
    case 0x98: // IC_ACK_GENERAL_CALL
    case 0x9C: // IC_ENABLE_STATUS
    case 0xA0: // IC_FS_SPKLEN
    case 0xA8: // IC_CLR_RESTART_DET
    case 0xF4: // IC_COMP_PARAM_1
    case 0xF8: // IC_COMP_VERSION
    case 0xFC: // IC_COMP_TYPE
      break;
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    std::cerr << "I2C::write_word_internal(0x" << std::hex << addr << ", 0x" << in << ")" << std::dec << std::endl;
    switch (addr & 0xff) {
    case 0x00: // IC_CON
    case 0x04: // IC_TAR  
    case 0x08: // IC_SAR  
    case 0x10: // IC_DATA_CMD  
    case 0x14: // IC_SS_SLC_HCNT  
    case 0x18: // IC_SS_SLC_LCNT  
    case 0x1c: // IC_FS_SLC_HCNT  
    case 0x20: // IC_FS_SLC_LCNT  
    case 0x2c: // IC_INTR_STAT  
    case 0x30: // IC_INTR_MASK  
    case 0x34: // IC_RAW_INTR_STAT  
    case 0x38: // IC_RX_TL  
    case 0x3C: // IC_TX_TL  
    case 0x40: // IC_CLR_INTR  
    case 0x44: // IC_CLR_RX_UNDER  
    case 0x48: // IC_CLR_RX_OVER  
    case 0x4C: // IC_CLR_TX_OVER  
    case 0x50: // IC_CLR_RD_REQ  
    case 0x54: // IC_CLR_TX_ABRT  
    case 0x58: // IC_CLR_RX_DONE  
    case 0x5C: // IC_CLR_ACTIVITY  
    case 0x60: // IC_CLR_STOP_DET  
    case 0x64: // IC_CLR_START_DET  
    case 0x68: // IC_CLR_GEN_CALL  
    case 0x6C: // IC_ENABLE
    case 0x70: // IC_STATUS
    case 0x74: // IC_TXFLR
    case 0x78: // IC_RXFLR
    case 0x7C: // IC_SDA_HOLD
    case 0x80: // IC_TX_ABRT_SOURCE
    case 0x84: // IC_SLV_DATA_NACK_ONLY
    case 0x88: // IC_DMA_CR
    case 0x8C: // IC_DMA_TDLR
    case 0x90: // IC_DMA_RDLR
    case 0x94: // IC_SDA_SETUP
    case 0x98: // IC_ACK_GENERAL_CALL
    case 0x9C: // IC_ENABLE_STATUS
    case 0xA0: // IC_FS_SPKLEN
    case 0xA8: // IC_CLR_RESTART_DET
    case 0xF4: // IC_COMP_PARAM_1
    case 0xF8: // IC_COMP_VERSION
    case 0xFC: // IC_COMP_TYPE
      break;
    default:
      return PortState::FAULT;
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {return 0;}

private:
  InterruptSourceMulti m_irqs;
  GPIOSignal m_sda;
  GPIOSignal m_scl;
};