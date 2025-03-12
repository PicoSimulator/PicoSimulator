#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include "platform/rpi/rp2040/peri/dma/dreq.hpp"
#include "clock.hpp"
#include "fifo.hpp"

#include <fstream>
#include <memory>
#include <bitset>

class UART final : public IPeripheralPort, public IClockable{
public:
  UART(const std::string &name, InterruptSource &irq_source)
  : m_irqs{irq_source}
  , m_vcd{name}
  {
    m_idiv = 0;
    m_fdiv = 0;
    m_control = 0;
    m_lcr_h = 0;
    m_lcr_l = 0;
    m_shift_in_counter = 0;
    m_shift_out_counter = 0;
    m_rx_fifo.enable(false);
    m_tx_fifo.enable(false);
    m_vcd.add_item(m_status);
    m_vcd.add_item(m_ibrd);
    m_vcd.add_item(m_fbrd);
    m_vcd.add_item(m_control);
    m_vcd.add_item(m_idiv);
    m_vcd.add_item(m_fdiv);
    m_vcd.add_item(m_char_out);
    m_vcd.add_item(m_char_in);
  }
  void tick() override final
  {
    if ((m_control & (int)Control::UART_EN) == 0) return;
    //fractional divider
    if (!m_idiv || (--m_idiv == 0 && m_fdiv > m_fbrd)) {
      m_idiv = m_ibrd;
      m_fbrd = (m_fdiv - m_fbrd) & 0x3f; // modulo 64
      subtick();
    }
  }
  void open(const std::string & path) {
    m_infile.rdbuf()->pubsetbuf(0, 0);
    m_infile.open(path, std::ios::binary | std::ios::out | std::ios::in);
    m_outfile.rdbuf()->pubsetbuf(0, 0);
    m_outfile.open(path, std::ios::binary | std::ios::out | std::ios::in);
  }
  RP2040::DMA::DReqSource &tx_dreq() { return m_tx_fifo.dreq(); }
  RP2040::DMA::DReqSource &rx_dreq() { return m_rx_fifo.dreq(); }
  enum class IRQ{
    RIM,
    CTSM,
    DCDM,
    DSRM,
    RX,
    TX,
    RT,
    FE,
    PE,
    BE,
    OE,
  };

  InterruptSource &get_irq(IRQ irq) { return m_irqs[(int)irq]; }

  GPIOSignal &TX() { return m_tx; }
  GPIOSignal &RX() { return m_rx; }
  GPIOSignal &CTS() { return m_cts; }
  GPIOSignal &RTS() { return m_rts; }

  Tracing::VCD::Module &vcd() { return m_vcd; }
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
          // m_status &= ~(1 << 4);
        }
        break;
      case UARTRSR: out = m_status; break;
      case UARTFR:
        out = (m_tx_fifo.full() << 5) | (m_tx_fifo.empty() << 7) | (m_rx_fifo.empty() << 4);
        break;
      case UARTIMSC:
        out = m_irqs.mask();
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
          if (m_tx_fifo.enabled() && m_tx_fifo.FiFoBase::count() == tx_fifo_threshold()) {
            
          }
        } else {
        }
        break;
      case UARTRSR: m_status &= ~in; break;
      case UARTFR: break; // READ ONLY REG
      case UARTIBRD: m_ibrd = in & 0x3fff; break;
      case UARTFBRD: m_fbrd = in & 0x3f; break;
      case UARTLCR_H: 
      {
        m_rx_fifo.enable(in & 0x10);
        m_tx_fifo.enable(in & 0x10);
        break;
      }
      case UARTCR:
        m_control = in;
        break;
      case UARTIFLS:
        m_uartifls = in;
        break;
      case UARTIMSC: m_irqs.mask(in); break;
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
    switch (addr & 0xff) {
      case UARTFR:
        return (m_tx_fifo.full() << 5) | (m_tx_fifo.empty() << 7) | (m_rx_fifo.empty() << 4);
      case UARTLCR_H:
        return m_lcr_h;
      case UARTCR:
        return m_control;
      case UARTIFLS:
        return 0;
      case UARTIMSC:
        return m_irqs.mask();
      case UARTIBRD:
        return m_ibrd;
      case UARTFBRD:
        return m_fbrd;
      case UARTPERIPHID0:
        return 0x11;
      case UARTPERIPHID1:
        return 0x10;
      case UARTPERIPHID2:
        return 0x34;
      case UARTPERIPHID3:
        return 0x00;
      case UARTPCELLID0:
        return 0x0d;
      case UARTPCELLID1:
        return 0xf0;
      case UARTPCELLID2:
        return 0x05;
      case UARTPCELLID3:
        return 0xb1;
    }
    assert(false);
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

  enum class Control{
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
    if (m_control & (int)Control::TX_EN && m_shift_out_counter == 0 && !m_tx_fifo.empty()) {
      m_shift_out_counter = wordlength();
      m_char_out = m_tx_fifo.pop();
      if (m_outfile.is_open()) {
        char c = m_char_out;
        m_outfile.write(&c, 1);
        m_outfile.flush();
      }
      if (!m_tx_fifo.enabled() && m_tx_fifo.empty()) {
        // there's only one space when disabled 
        // so that empty check is a bit unnecessary
        get_irq(IRQ::TX).raise();
      } else if(m_tx_fifo.FiFoBase::count() == tx_fifo_threshold()) {
        get_irq(IRQ::TX).raise();
      }
    } else if (m_shift_out_counter > 0) {
      m_shift_out_counter--;
    }
    if (m_control & (int)Control::RX_EN && m_shift_in_counter == 0) {
      m_shift_in_counter = wordlength();
      char c;
      if (m_infile.is_open()) {
        if (m_infile.readsome(&c, 1) == 1) {
          // check for overrun
          if (!m_rx_fifo.full()) {
            m_rx_fifo.push(c);
          }
          else{
            get_irq(IRQ::OE).raise();
            m_status |= 1 << 3;
          }
          if (!m_rx_fifo.enabled() && m_rx_fifo.full()) {
            // again that full check is a bit redundant
            get_irq(IRQ::RX).raise();
          } else if (m_rx_fifo.FiFoBase::count() == rx_fifo_threshold()) {
            get_irq(IRQ::RX).raise();
          }
          m_shift_in_counter = wordlength();
        } else {
          m_infile.sync();
        }
      }
    } else if (m_shift_in_counter > 0) {
      m_shift_in_counter--;
    }
    // std::cout << m_shift_in_counter << std::endl;;
    // std::cout << m_shift_out_counter << std::endl;;
  }

  uint8_t rx_fifo_threshold() const
  {
    return (std::array<uint8_t, 5>{4, 8, 16, 24, 28})[rxiflsel()];
  }

  uint8_t tx_fifo_threshold() const
  {
    return (std::array<uint8_t, 5>{4, 8, 16, 24, 28})[txiflsel()];
  }

  uint8_t rxiflsel() const
  {
    return (m_uartifls >> 3) & 0x07;
  }

  uint8_t txiflsel() const
  {
    return m_uartifls & 0x07;
  }

  uint32_t wordlength() const
  {
    return (m_lcr_h >> 5) & 3;
  }

  DReqTxFiFo<uint32_t, 32> m_tx_fifo;
  DReqRxFiFo<uint32_t, 32> m_rx_fifo;

  Tracing::VCD::Register<uint32_t> m_status{"status"};
  Tracing::VCD::Register<uint32_t> m_ibrd{"ibrd"};
  Tracing::VCD::Register<uint32_t> m_fbrd{"fbrd"};
  Tracing::VCD::Register<uint32_t> m_control{"control"};
  Tracing::VCD::Module m_vcd;

  uint32_t m_lcr_h;
  uint32_t m_lcr_l;

  Tracing::VCD::Register<uint32_t> m_idiv{"idiv"};
  Tracing::VCD::Register<uint32_t> m_fdiv{"fdiv"};

  std::ofstream m_outfile;
  std::ifstream m_infile;

  uint32_t m_shift_in_counter;
  uint32_t m_shift_out_counter;
  Tracing::VCD::Register<uint8_t> m_char_out{"cout"};
  Tracing::VCD::Register<uint8_t> m_char_in{"cin"};

  uint32_t m_uartifls;

  InterruptSourceMulti m_irqs;

  GPIOSignal m_tx, m_rx, m_cts, m_rts;

};