#pragma once

#include <cstdint>
#include <strings.h>
#include <iostream>
#include <iomanip>

#include "net.hpp"
#include "device.hpp"

#define SEND_BUFFER_MAX 1024

class SPIDev : public IODevice {
public:
  SPIDev()
  : IODevice()
  , m_cs_pin{*this} 
  , m_sclk_pin{*this}
  {
    add_named_pin("~CS", m_cs_pin);
    add_named_pin("SCLK", m_sclk_pin);
    add_named_pin("MOSI", m_mosi_pin);
    add_named_pin("MISO", m_miso_pin);
    m_send_buffer_index = 0;
    m_send_buffer_len = 0;
  }
  NetConnection &cs_pin() { return m_cs_pin; }

  virtual void spi_cs_changed(bool cs) = 0;
  virtual void spi_done(uint8_t *data, size_t len)  = 0;

  uint8_t spi_exchange_bit(uint8_t in)
  {
    // std::cerr << "spi_exchange_bit: "  << int(in) << std::endl;
    if (m_shiftpos <= 0) {
      if (m_send_buffer_index < m_send_buffer_len)
        m_sr_out = m_send_buffer[m_send_buffer_index];
      m_shiftpos = s_shiftcnt_bits;
    }
    uint8_t ret = spi_peek_bit();
    spi_advance_bit(in);
    return ret;
  }
  uint8_t spi_peek_bit()
  {
    return m_sr_out & 0x80;
  }

  void spi_advance_bit(uint8_t in)
  {
    m_sr_in <<= 1;
    m_sr_in |= in;
    m_sr_out <<= 1;
    m_shiftpos--;
    if (m_shiftpos == 0) {
      if (m_send_buffer_index < m_send_buffer_len) {
        m_send_buffer[m_send_buffer_index] = m_sr_in;
        m_send_buffer_index++;
        if (m_send_buffer_index == m_send_buffer_len) {
          spi_done(m_send_buffer, m_send_buffer_index);
        }
      }
    }
  }

  uint8_t spi_dual_read_bits()
  {
    // rather hackish imo
    uint8_t in = 0;
    uint8_t out = 0;
    out |= spi_exchange_bit(0);
    out |= spi_exchange_bit(0) << 1;
    return out;
  }

  uint8_t spi_quad_read_bits()
  {
    // rather hackish imo
    uint8_t in = 0;
    uint8_t out = 0;
    out |= spi_exchange_bit(0);
    out |= spi_exchange_bit(0) << 1;
    out |= spi_exchange_bit(0) << 2;
    out |= spi_exchange_bit(0) << 3;
    return out;
  }

  void spi_dual_write_bits(uint8_t data)
  {
    spi_exchange_bit((data >> 0) & 1);
    spi_exchange_bit((data >> 1) & 1);
  }

  void spi_quad_write_bits(uint8_t data)
  {
    spi_exchange_bit((data >> 0) & 1);
    spi_exchange_bit((data >> 1) & 1);
    spi_exchange_bit((data >> 2) & 1);
    spi_exchange_bit((data >> 3) & 1);
  }


  uint8_t spi_exchange_byte(uint8_t in)
  {
    std::cerr << "spi_exchange_byte: " << std::hex << std::setw(2) << std::setfill('0')  << int(in) << std::dec << std::endl;
    uint8_t out = 0;
    if (m_send_buffer_index < m_send_buffer_len) {
      out = m_send_buffer[m_send_buffer_index];
      m_send_buffer[m_send_buffer_index] = in;
      m_send_buffer_index++;
      std::cout << "spi_exchange_byte: " << std::hex << std::setw(2) << std::setfill('0')  << int(in) << "/" << int(out) << std::dec << std::endl;
    } else {
      std::cout << "m_send_buffer_index: " << m_send_buffer_index << std::endl;
      std::cout << "m_send_buffer_len: " << m_send_buffer_len << std::endl;
    }
    if (m_send_buffer_index == m_send_buffer_len) {
      std::cout << "spi_done" << std::endl;
      spi_done(m_send_buffer, m_send_buffer_index);
    }
    return out;
  }

  void qspi_write(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      spi_exchange_byte(data[i]);
    }
  }
  void qspi_recieve(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      data[i] = spi_exchange_byte(0);
    }
  }

  void set_cs(bool cs) 
  {
    if (m_cs != cs) {
      m_cs = cs;
      m_shiftpos = s_shiftcnt_bits;
      if (cs)
        spi_done(m_send_buffer, m_send_buffer_index);
      spi_cs_changed(cs);
    }
  }

protected:
  void spi_start(uint8_t *data, size_t len)
  {
    std::cout << "spi_start: " << (void*)(data) << "/" << len << std::endl;
    m_send_buffer_len = len;
    for(size_t i = 0; i < len; i++)
      m_send_buffer[i] = data?data[i]:0;
    m_send_buffer_index = 0;
    m_shiftpos = s_shiftcnt_bits;
  }
  void spi_stop()
  {
    m_send_buffer_len = 0;
    m_send_buffer_index = 0;
  }
  void qspi_start(uint8_t *data, size_t len)
  {
    spi_start(data, len);
  }
  void qspi_stop()
  {
    spi_stop();
  }
  
  bool spi_cs() const { return m_cs; }

  void set_miso_enabled(bool en) {} 
private:
  uint8_t m_send_buffer[SEND_BUFFER_MAX];
  size_t m_send_buffer_len;
  size_t m_send_buffer_index;
  bool m_cs;
  uint8_t m_sr_in, m_sr_out;
  uint8_t m_shiftpos;

  static constexpr size_t s_shiftcnt_bits = 8;

  class CS_Pin final : public NetConnection{
  public:
    CS_Pin(SPIDev &dev) : m_dev{dev}{}
    void net_state_changed() override{
      bool val = connected_net()->digital_read();
      m_dev.set_cs(val);
      m_dev.m_miso_pin.set_drive_strength(val?0:255);
      m_dev.m_miso_pin.set_drive_value(m_dev.spi_peek_bit());
    }
  protected:
  private:
    SPIDev &m_dev;
  } m_cs_pin;
  class SCLK_Pin final : public NetConnection{
  public:
    SCLK_Pin(SPIDev &dev) 
    : m_dev{dev}
    , m_prev_val{false}
    {}
    void net_state_changed() override{
      if (m_dev.spi_cs() == 1)
        return;
      bool val = connected_net()->digital_read();
      bool prev_val = m_prev_val;
      m_prev_val = val;
      // no clock transition
      if (val == prev_val) {
        return;
      }
      if (val ^ m_dev.m_cpol ^ m_dev.m_cpha) {
        // transition from idle clock state
        bool bit_in = 0;
        if (m_dev.m_mosi_pin.is_connected()) {
          bit_in = m_dev.m_mosi_pin.connected_net()->digital_read();
        }
        m_bit_out = m_dev.spi_exchange_bit(bit_in);
      } else {
        // transition to idle clock state
        m_dev.m_miso_pin.set_drive_value(m_bit_out);
        // no transition
        return;
      }
    }
  protected:
  private:
    bool m_prev_val;
    bool m_bit_out;
    SPIDev &m_dev;
  } m_sclk_pin;
  NetConnection m_mosi_pin;
  NetConnection m_miso_pin;
  bool m_cpol;
  bool m_cpha;
};