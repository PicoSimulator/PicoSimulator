#pragma once

#include "rp2040/peripheral.hpp"
#include "clock.hpp"
#include "fifo.hpp"

class UART final : public IPeripheralPort, public IClockable{
public:
  void tick() override final
  {
    //fractional divider
    if (!m_idiv || (--m_idiv == 0 && m_fdiv > m_fbrd)) {
      m_idiv = m_ibrd;
      m_fbrd = (m_fdiv - m_fbrd) & 0x3f; // modulo 64
      subtick();
    }
  }
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    switch(addr & 0xff) {
      case UARTDR:
        if (!m_rx_fifo.empty()) {
          out = m_rx_fifo.pop();
        } else {
          out = 0;
          m_status &= ~(1 << 4);
        }
        break;
      case UARTIBRD: out = m_ibrd; break;
      case UARTFBRD: out = m_fbrd; break;
      case UARTPERIPHID0: out = 0x11; break;
      case UARTPERIPHID1: out = 0x10; break;
      case UARTPERIPHID2: out = 0x34; break;
      case UARTPERIPHID3: out = 0x00; break;
      case UARTPCELLID0: out = 0x0d; break;
      case UARTPCELLID1: out = 0xf0; break;
      case UARTPCELLID2: out = 0x05; break;
      case UARTPCELLID3: out = 0xb1; break;
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
    if ((addr & 0xff) == 0x00) {
      std::cout << "UART DR: " << std::hex << in << "  (" << char(in) << ")" << std::endl;
    }
    switch(addr & 0xff) {
      case UARTDR:
        if (!m_tx_fifo.full()) {
          m_tx_fifo.push(in);
        }
        break;
      case UARTIBRD: m_ibrd = in & 0x3fff; break;
      case UARTFBRD: m_fbrd = in & 0x3f; break;
      case UARTLCR_H: break;
      case UARTCR:
        if (in & 1) {
          m_status |= 1;
        } else {
          m_status &= ~1;
        }
        break;
      case UARTICR:
        m_status &= ~in;
        break;
      case UARTDMACR:
        break;
    }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

private:
  enum Register{
    UARTDR = 0x00,
    UARTRSR = 0x04,
    UARTFR = 0x18,
    UARTILPR = 0x20,
    UARTIBRD = 0x24,
    UARTFBRD = 0x28,
    UARTLCR_H = 0x2c,
    UARTCR = 0x30,
    UARTIFLS = 0x34,
    UARTIMSC = 0x38,
    UARTRIS = 0x3c,
    UARTMIS = 0x40,
    UARTICR = 0x44,
    UARTDMACR = 0x48,
    UARTPERIPHID0 = 0xfe0,
    UARTPERIPHID1 = 0xfe4,
    UARTPERIPHID2 = 0xfe8,
    UARTPERIPHID3 = 0xfec,
    UARTPCELLID0 = 0xff0,
    UARTPCELLID1 = 0xff4,
    UARTPCELLID2 = 0xff8,
    UARTPCELLID3 = 0xffc

  };
  void subtick() {
    std::cout << "UART_SUBTICK" << std::endl;
  }

  FiFo<uint32_t, 32> m_tx_fifo;
  FiFo<uint32_t, 32> m_rx_fifo;

  uint32_t m_status;
  uint32_t m_ibrd;
  uint32_t m_fbrd;
  uint32_t m_control;

  uint32_t m_idiv;
  uint32_t m_fdiv;

};