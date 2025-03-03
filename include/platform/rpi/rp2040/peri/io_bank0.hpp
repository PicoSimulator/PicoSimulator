#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include <array>
#include <functional>
#include "platform/rpi/rp2040/gpio.hpp"

class IOBank0 final : public IPeripheralPort{
public:
  IOBank0(std::array<std::reference_wrapper<RP2040::GPIO>, 30> bank0_gpios)
  : m_gpios{bank0_gpios}
  {}
protected:
  virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override final
  {
    out = 0;
    if ((addr & 0x1f0) <= 0xe0) {
      uint8_t gpio_num = (addr >> 3) & 0x7;
      auto &gpio = m_gpios[gpio_num].get();
      switch (addr & 0x04) {
        case 0x00: out = gpio.get_status(); break;
        case 0x04: out = gpio.get_ctrl(); break;
      }
    } else if (addr & 0x1f0 == 0x1f0) {
      // INTR
    }
    return PortState::SUCCESS;
  }
  virtual PortState write_word_internal(uint32_t addr, uint32_t in) override final
  {
      if ((addr & 0x1f0) <= 0xe0) {
        uint8_t gpio_num = (addr >> 3) & 0x1f;
        auto &gpio = m_gpios[gpio_num].get();
        switch (addr & 0x04) {
          case 0x00: break;
          case 0x04: gpio.set_ctrl(in); break;
        }
      } else if (addr & 0x1f0 == 0x1f0) {
        // INTR
      }
    return PortState::SUCCESS;
  }
  virtual uint32_t read_word_internal_pure(uint32_t addr) const override final
  {
    return 0;
  }

private:
  std::array<std::reference_wrapper<RP2040::GPIO>, 30> m_gpios;
};