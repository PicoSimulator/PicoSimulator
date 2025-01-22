#pragma once

#include "ext/io/spidev.hpp"
#include <array>
#include <span>

class W25QFlash final : public SPIDev
{
public:
  W25QFlash() : SPIDev{} {

    spi_start(nullptr, 1);
  }
  virtual ~W25QFlash() {}

  std::span<uint8_t> flash() { return m_flash; }
  std::span<const uint8_t> flash() const { return m_flash; }

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
          case Command::WriteEnable: m_write_enabled = true; spi_start(nullptr, 1); break;
          case Command::WriteDisable: m_write_enabled = false; spi_start(nullptr, 1); break;
          case Command::SectorErase: spi_start(nullptr, 3); m_state = State::ERASE_ADDR; m_erase_size = 0x1000; break;
          case Command::ReadSR1: spi_start(nullptr, 1); m_state=State::NEXTCMD; break;
          case Command::FastReadQuadIO: spi_start(nullptr, 6); m_state = State::FAST_ADDR; break;
          case Command::PageProgram: spi_start(nullptr, 3); m_state = State::PGM_ADDR; break;
          case Command::WriteStatusRegister: spi_start(nullptr, 2); m_state = State::NEXTCMD; break;
          default: spi_start(nullptr, 1); break;
        }
        break;
      }
      case State::NEXTCMD: {
        m_state = State::CMD;
        spi_start(nullptr, 1);
        break;
      }
      case State::ERASE_ADDR: {
        m_addr = (data[0] << 16) | (data[1] << 8) | (data[2]);
        m_addr &= ~(m_erase_size - 1);
        if (m_write_enabled) {
          for (size_t i = 0; i < m_erase_size; i ++) {
            m_flash[m_addr + i] = s_erased_byte_value;
          }
          std::cout << "Erased " << std::hex << m_erase_size << " bytes at " << m_addr << std::dec << "\n";
        }
        if(spi_cs()) {
          spi_stop();
        }
        spi_start(nullptr, 1);
        m_state = State::CMD;
      } break;
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
      } break;
      case State::FAST_ADDR: {
        m_addr = (data[0] << 16) | (data[1] << 8) | (data[2]);
        m_state = State::DATA;
        if(spi_cs()) {
          spi_stop();
        }
        spi_start(&m_flash[m_addr], 32);
        m_addr += 32;
      } break;
      case State::PGM_ADDR: {
        m_addr = (data[0] << 16) | (data[1] << 8) | (data[2]);
        m_state = State::PGM_DATA;
        spi_start(&m_page_buffer[0], 256);
      } break;
      case State::PGM_DATA: {
        std::cout << "Programming " << len << " bytes at " << std::hex << m_addr << "\n";
        for (size_t i = 0; i < len; i++) {
          m_flash[m_addr+i] &= data[i];
          std::cout << std::hex << std::setfill('0') << std::setw(2) << int(m_flash[m_addr+i]) << " ";
        }
        std::cout << std::dec << "\n";
        if(spi_cs()) {
          spi_stop();
        }
        spi_start(nullptr, 1);
        m_state = State::CMD;
      }
    }
  }

  void load_binary_data(const std::span<uint8_t> data)
  {
    std::copy(data.begin(), data.end(), m_flash.begin());
  }
private:
  enum State{
    CMD,
    ADDR,
    DATA,
    ERASE_ADDR,
    NEXTCMD,
    FAST_ADDR,
    PGM_ADDR,
    PGM_DATA,
  } m_state;

  enum Command : uint8_t{
    WriteStatusRegister = 0x01,
    PageProgram = 0x02,
    ReadData = 0x03,
    WriteDisable = 0x04,
    ReadSR1 = 0x05,
    WriteEnable = 0x06,
    FastRead = 0x0b,
    SectorErase = 0x20,
    ReadSR2 = 0x35,
    FastReadDualOutput = 0x3b,
    FastReadQuadOutput = 0x6b,
    SetBustWithWrap = 0x77,
    FastReadDualIO = 0xbb,
    FastReadQuadIO = 0xeb,
  };
  Command m_current_command;
  uint32_t m_addr;
  uint32_t m_erase_size;
  static constexpr uint8_t s_erased_byte_value = 0xff;

  std::array<uint8_t, 0x0100'0000> m_flash;
  std::array<uint8_t, 256> m_page_buffer;
  bool m_write_enabled;

};