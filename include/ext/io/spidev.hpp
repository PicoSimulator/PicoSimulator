#pragma once

#include <cstdint>
#include <strings.h>

#define SEND_BUFFER_MAX 64

class SPIDev {
public:

  virtual void spi_cs_changed(bool cs) = 0;
  virtual void spi_done(uint8_t *data, size_t len)  = 0;

  uint8_t spi_exchange_byte(uint8_t in)
  {
    uint8_t out = 0;
    if (m_send_buffer_index < m_send_buffer_len) {
      out = m_send_buffer[m_send_buffer_index];
      m_send_buffer[m_send_buffer_index] = in;
      m_send_buffer_index++;
      std::cout << "spi_exchange_byte: " << int(in) << "/" << int(out) << std::endl;
    }
    if (m_send_buffer_index == m_send_buffer_len) {
      std::cout << "spi_done" << std::endl;
      spi_done(m_send_buffer, m_send_buffer_index);
    }
    return out;
  }

  void set_cs(bool cs) 
  {
    if (m_cs != cs) {
      m_cs = cs;
      if (cs)
        spi_done(m_send_buffer, m_send_buffer_index);
      spi_cs_changed(cs);
    }
  }

protected:
  void spi_start(uint8_t *data, size_t len)
  {
    std::cout << "spi_start: " << uintptr_t(data) << "/" << len << std::endl;
    m_send_buffer_len = len;
    for(size_t i = 0; i < len; i++)
      m_send_buffer[i] = data?data[i]:0;
    m_send_buffer_index = 0;
  }
  void spi_stop()
  {
    m_send_buffer_len = 0;
    m_send_buffer_index = 0;
  }
  bool spi_cs() const { return m_cs; }
private:
  uint8_t m_send_buffer[SEND_BUFFER_MAX];
  size_t m_send_buffer_len;
  size_t m_send_buffer_index;
  bool m_cs;
};