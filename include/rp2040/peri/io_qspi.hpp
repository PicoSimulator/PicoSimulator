#pragma once

#include "rp2040/peripheral.hpp"
#include "rp2040/gpio.hpp"
#include "ext/io/spidev.hpp"

#include <iostream>

  namespace RP2040{
  class IOQSPI final : public IPeripheralPort{
  public:
    IOQSPI(std::array<std::reference_wrapper<GPIO>, 6> gpios)
    : m_gpios{gpios} {}
    void set_spidev(SPIDev *spidev)
    {
      this->spidev = spidev;
    }
    GPIO &get_gpio(uint8_t n) { return m_gpios[n]; }
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
    {
      out = 0;
      if ((addr & 0x70) <= 0x20) {
        uint8_t gpio_num = (addr >> 3) & 0x7;
        auto &gpio = m_gpios[gpio_num].get();
        switch (addr & 0x04) {
          case 0x00: out = gpio.get_status(); break;
          case 0x04: out = gpio.get_ctrl(); break;
        }
      } else if (addr & 0x7c == 0x30) {
        // INTR
      }
      return PortState::SUCCESS;
    }
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
    {
      if ((addr & 0x30) <= 0x20) {
        uint8_t gpio_num = (addr >> 3) & 0x7;
        auto &gpio = m_gpios[gpio_num].get();
        switch (addr & 0x04) {
          case 0x00: break;
          case 0x04: gpio.set_ctrl(in); break;
        }
      } else if (addr & 0xfc == 0x30) {
        // INTR
      }
      return PortState::SUCCESS;
    }
    virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
    {
      return 0;
    }

  private:
    SPIDev *spidev;
    std::array<std::reference_wrapper<GPIO>, 6> m_gpios;
  };
}