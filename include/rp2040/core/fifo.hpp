#pragma once

#include <array>

namespace RP2040::Core
{
  
  class FiFo{
  public:
    void send(uint32_t data) { 
      if(m_count < 8){
        m_data[(m_head + m_count) % 8] = data;
        m_count++;
      }else{
        wof = true;
      }
    }
    uint32_t recv() { 
      if(m_count > 0){
        uint32_t data = m_data[m_head];
        m_head = (m_head + 1) % 8;
        m_count--;
        return data;
      }else{
        roe = true;
        return 0;
      }
    }
    uint32_t status() const { return status_send() | status_recv(); }
    uint32_t status_send() const { 
      return m_count < 8? 2 : 0;
    }
    uint32_t status_recv() const { 
      return m_count > 0 ? 1 : 0;
    }
    void clear_send() { wof = false; }
    void clear_recv() { roe = false; }
  protected:
  private:
    std::array<uint32_t, 8> m_data;
    uint8_t m_head = 0;
    uint8_t m_count = 0;
    bool wof, roe;
  };
} // namespace RP2040::Core
