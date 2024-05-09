#pragma once

#include <cstdint>

class Divider{
public:
  bool tick(){
    if(m_cnt == 0){
      m_cnt = m_divisor;
      return true;
    }
    m_cnt--;
    return false;
  }
protected:
private:
  uint32_t m_divisor;
  uint32_t m_cnt;
};