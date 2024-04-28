#pragma once

#include "ext/io/spidev.hpp"
#include <array>
#include <fstream>

class W25QFlash final : public SPIDev
{
public:
  W25QFlash() : SPIDev{} {
    // for (size_t i = 0; i < m_flash.size(); i++)
      // m_flash[i] = i & 0xff;

    std::ifstream file("/home/skyler/git/raspberrypi/pico-examples/build/hello_world/serial/hello_serial.bin", std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file.read((char*)m_flash.begin(), size))
    {
        /* worked! */
    } else {
        /* failed! */
        std::terminate();
    }

    spi_start(nullptr, 1);
  }
  virtual ~W25QFlash() {}

  virtual void spi_cs_changed(bool cs)
  {
    if (cs == 0) {
      spi_start(nullptr, 1);
      m_state = State::CMD;
    }
  }

  virtual void spi_done(uint8_t *data, size_t len)
  {
    switch(m_state) {
      case State::CMD: {
        m_current_command = Command(data[0]);
        switch(m_current_command) {
          case Command::ReadData: spi_start(nullptr, 3); m_state = State::ADDR; break;
          case Command::FastRead: spi_start(nullptr, 4); m_state = State::ADDR; break;
          default: spi_start(nullptr, 1); break;
        }
        break;
      }
      case State::ADDR: {
        m_addr = (data[0] << 16) | (data[1] << 8) | (data[2]);
        m_state = State::DATA;
      }
      [[fallthrough]];
      case State::DATA: {
        if(spi_cs()) {
          spi_stop();
        }
        spi_start(&m_flash[m_addr], 32);
        m_addr += 32;
      }
    }
  }
private:
  enum State{
    CMD,
    ADDR,
    DATA,
  } m_state;

  enum Command : uint8_t{
    ReadData = 0x03,
    FastRead = 0x0b,
  };
  Command m_current_command;
  uint32_t m_addr;

  std::array<uint8_t, 0x0100'0000> m_flash;

};