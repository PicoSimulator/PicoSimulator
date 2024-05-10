#pragma once

#include "rp2040/peripheral.hpp"
#include "rp2040/peri/dma/dreq.hpp"
#include "clock.hpp"
#include "fifo.hpp"

#include <fstream>
#include <memory>
#include <bitset>

class UART final : public IPeripheralPort, public IClockable{
public:
  UART()
  {
    m_idiv = 0;
    m_fdiv = 0;
    m_control = 0;
    m_lcr_h = 0;
    m_lcr_l = 0;
    m_shift_in_counter = 0;
    m_shift_out_counter = 0;
  }
  void tick() override final
  {
    if ((m_control & Control::UART_EN) == 0) return;
    //fractional divider
    if (!m_idiv || (--m_idiv == 0 && m_fdiv > m_fbrd)) {
      m_idiv = m_ibrd;
      m_fbrd = (m_fdiv - m_fbrd) & 0x3f; // modulo 64
      subtick();
    }
  }
  void open(const std::string & path) {
    m_file.rdbuf()->pubsetbuf(0, 0);
    m_file.open(path, std::ios::binary | std::ios::out | std::ios::in);
    if (m_file.badbit)
      std::cout << "Failed to open file" << std::endl;
  }
  RP2040::DMA::DReqSource &tx_dreq() { return m_tx_fifo.dreq(); }
  RP2040::DMA::DReqSource &rx_dreq() { return m_rx_fifo.dreq(); }
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
      case UARTCR: out = m_control; break;
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
    switch(addr & 0xff) {
      case UARTDR:
        if (!m_tx_fifo.full()) {
          m_tx_fifo.push(in);
          std::cout << "UART DR: (" << char(in) << ")" << std::endl;
        } else {
          std::cout << "UART TX FIFO OVERFLOW!" << std::endl;
        }
        break;
      case UARTIBRD: m_ibrd = in & 0x3fff; break;
      case UARTFBRD: m_fbrd = in & 0x3f; break;
      case UARTLCR_H: break;
      case UARTCR:
        m_control = in;
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

  enum Control{
    UART_EN = 1<<0,
    SIR_EN = 1<<1,
    SIR_LP = 1<<2,
    LOOPBACK_EN = 1<<7,
    TX_EN = 1<<8,
    RX_EN = 1<<9,
    DTR = 1<<10,
    RTS = 1<<11,
    OUT1 = 1<<12,
    OUT2 = 1<<13,
    RTS_EN = 1<<14,
    CTS_EN = 1<<15,
  };

  void subtick() {
    // std::cout << "UART_SUBTICK" << std::endl;
    // std::cout << std::bitset<32>{m_control} << std::endl;
    if (m_control & Control::TX_EN && m_shift_out_counter == 0 && !m_tx_fifo.empty()) {
      m_shift_out_counter = wordlength();
      char c = m_tx_fifo.pop();
      if (m_file.is_open()) {
        m_file.write(&c, 1);
        m_file.flush();
      }
    } else if (m_shift_out_counter > 0) {
      m_shift_out_counter--;
    }
    if (m_control & Control::RX_EN && m_shift_in_counter == 0) {
      m_shift_in_counter = wordlength();
      char c;
      if (m_file.is_open()) {
        if (m_file.readsome(&c, 1) == 1) {
          std::cout << "UART CHAR RECEIVED: " << c << std::endl;
          // check for overflow
          m_rx_fifo.push(c);
          m_shift_in_counter = wordlength();
        }
      }
    } else if (m_shift_in_counter > 0) {
      m_shift_in_counter--;
    }
  }

  uint32_t wordlength() const
  {
    return (m_lcr_h >> 5) & 3;
  }

  DReqTxFiFo<uint32_t, 32> m_tx_fifo;
  DReqRxFiFo<uint32_t, 32> m_rx_fifo;

  uint32_t m_status;
  uint32_t m_ibrd;
  uint32_t m_fbrd;
  uint32_t m_control;

  uint32_t m_lcr_h;
  uint32_t m_lcr_l;

  uint32_t m_idiv;
  uint32_t m_fdiv;

  std::fstream m_file;

  uint32_t m_shift_in_counter;
  uint32_t m_shift_out_counter;

};