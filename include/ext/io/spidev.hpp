#pragma once

#include <cstdint>
#include <strings.h>
#include <iostream>
#include <iomanip>

#include "net.hpp"

#define SEND_BUFFER_MAX 1024

class SPIDev {
public:
  SPIDev()
  : m_cs_pin{*this} {}
  NetConnection &cs_pin() { return m_cs_pin; }

  virtual void spi_cs_changed(bool cs) = 0;
  virtual void spi_done(uint8_t *data, size_t len)  = 0;

  uint8_t spi_exchange_bit(uint8_t in)
  {
    if (m_shiftpos <= 0) {
      if (m_send_buffer_index < m_send_buffer_len)
        m_sr_out = m_send_buffer[m_send_buffer_index];
      m_shiftpos = s_shiftcnt_bits;
    }
    m_sr_in <<= 1;
    m_sr_in |= in;
    uint8_t ret = m_sr_out & 0x80;
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
    return ret;
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
    std::cout << "spi_exchange_byte: " << std::hex << std::setw(2) << std::setfill('0')  << int(in) << std::dec << std::endl;
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
    std::cout << "set_cs: " << cs << std::endl;
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
private:
  uint8_t m_send_buffer[SEND_BUFFER_MAX];
  size_t m_send_buffer_len;
  size_t m_send_buffer_index;
  bool m_cs;
  uint8_t m_sr_in, m_sr_out;
  uint8_t m_shiftpos;

  static constexpr size_t s_shiftcnt_bits = 7;

  class CS_Pin final : public NetConnection{
  public:
    CS_Pin(SPIDev &dev) : m_dev{dev}{}
    void net_state_changed() override{
      bool val = connected_net()->digital_read();
      // std::cerr << "CS_Pin::net_state_changed cs=" << val << "\n";
      m_dev.set_cs(val);
    }
  protected:
  private:
    SPIDev &m_dev;
  } m_cs_pin;
};