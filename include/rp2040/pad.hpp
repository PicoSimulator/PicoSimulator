#pragma once

#include <cstdint>
#include "ext/io/net.hpp"

namespace RP2040 {

  class Pad;

  class GPIOSignal{
  public:
    bool get_output() const { return m_output_value; }
    bool get_oe() const { return m_output_enable; }
    void set_output(bool value);
    virtual void irq() {}
    virtual void input_changed() {}
  protected:
  private:
    bool m_output_enable;
    bool m_output_value;
    Pad *m_current_pad;
  };

  class Pad final : public NetConnection {
  public:

    enum SlewRate{
      SLEW_FAST,
      SLEW_SLOW
    };

    enum DriveStrength{
      DRIVE_2MA,
      DRIVE_4MA,
      DRIVE_8MA,
      DRIVE_12MA
    };

    enum InputHysteresis{
      HYSTERESIS_OFF,
      HYSTERESIS_ON
    };

    enum Override{
      OVERRIDE_NONE,
      OVERRIDE_INV,
      OVERRIDE_LOW,
      OVERRIDE_HIGH,
    };

    bool get_output() const { return apply_override(m_output_override, m_output_value_raw); }
    bool get_input() const { return apply_override(m_input_override, m_input_value_raw); }

    void set_slew_rate(SlewRate);
    SlewRate get_slew_rate() const; 
    void set_drive_strength(DriveStrength);
    DriveStrength get_drive_strength() const;
    void set_input_hysteresis(InputHysteresis);
    InputHysteresis get_input_hysteresis() const;

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

    void set_status(uint32_t);
    uint32_t get_status() const;

    void update_from_mux();

  protected:
    virtual void net_state_changed() override;
    void pad_state_changed();
    static bool apply_override(Override o, bool val){
      switch(o){
        case OVERRIDE_NONE: return val;
        case OVERRIDE_INV: return !val;
        case OVERRIDE_LOW: return false;
        case OVERRIDE_HIGH: return true;
      }
    }
  private:
    GPIOSignal *m_gpio_funcs[9];
    uint8_t m_funcsel;
    bool m_input_value_raw;
    Override m_input_override;
    bool m_output_value_raw;
    Override m_output_override;
    bool m_output_enable_raw;
    Override m_oe_override;
  };

}