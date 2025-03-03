#pragma once

#pragma once

#include <cstdint>
#include <array>
#include <functional>
#include <cassert>
#include "platform/rpi/rp2040/pad.hpp"
#include "common/gpio.hpp"

namespace RP2040 {

  class GPIO;
  class Pad;


  class GPIO final : public ::GPIO {
  public:
    GPIO(Pad &pad, std::array<std::reference_wrapper<GPIOSignal>,32> funcs)
    : m_gpio_funcs{funcs}
    , m_pad{pad}
    // , m_output_enable_raw(1)
    {}

    enum Override{
      OVERRIDE_NONE,
      OVERRIDE_INV,
      OVERRIDE_LOW,
      OVERRIDE_HIGH,
    };

    bool get_output() const override { return apply_override(m_output_override, current_signal().get_output()); }
    bool get_output_enable() const override { return apply_override(m_oe_override, current_signal().get_oe()); }
    bool get_input() const override { return apply_override(m_input_override, m_pad.digital_read()); }

    void set_oe_override(Override);
    Override get_oe_override() const;
    void set_output_override(Override);
    Override get_output_override() const;
    void set_input_override(Override);
    Override get_input_override() const;
    void set_irq_override(Override);
    Override get_irq_override() const;

    void set_funcsel(uint8_t);
    uint8_t get_funcsel() const;

    void set_ctrl(uint32_t);
    uint32_t get_ctrl() const;

    uint32_t get_status() const;

    void update_from_internal() override;

  protected:
    void pad_state_changed();
    static bool apply_override(Override o, bool val){
      switch(o){
        case OVERRIDE_NONE: return val;
        case OVERRIDE_INV: return !val;
        case OVERRIDE_LOW: return false;
        case OVERRIDE_HIGH: return true;
      }
      assert(false);
      return false;
    }
  private:
    GPIOSignal &current_signal() { return m_gpio_funcs[m_funcsel].get(); }
    const GPIOSignal &current_signal() const { return m_gpio_funcs[m_funcsel].get(); }
    std::array<std::reference_wrapper<GPIOSignal>, 32> m_gpio_funcs;
    uint8_t m_funcsel;
    // bool m_input_value_raw;
    // bool m_output_value_raw;
    // bool m_output_enable_raw;
    Override m_input_override;
    Override m_output_override;
    Override m_oe_override;
    Override m_irq_override;
    Pad &m_pad;
  };

}