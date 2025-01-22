#pragma once

#include <cstdint>
#include "ext/io/net.hpp"

namespace RP2040 {

  class Pad;
  class GPIO;

  class Pad final : public NetConnection {
  public:
    Pad(GPIO &gpio)
    : m_gpio{gpio}
    {}

    enum SlewRate{
      SLEW_FAST,
      SLEW_SLOW
    };

    enum DriveStrength{
      DRIVE_2MA, // 1kohm  (96 ~= 127/3*4)
      DRIVE_4MA, // 500ohm (86)
      DRIVE_8MA, // 250ohm (76)
      DRIVE_12MA // 167ohm (70)
    };

    enum InputHysteresis{
      HYSTERESIS_OFF,
      HYSTERESIS_ON
    };

    void update();

    void set_slew_rate(SlewRate);
    SlewRate get_slew_rate() const; 
    void set_drive_strength(DriveStrength);
    DriveStrength get_drive_strength() const { return m_drive_strength; }
    void set_input_hysteresis(InputHysteresis);
    InputHysteresis get_input_hysteresis() const;
    void set_pullup_enable(bool);
    void set_pulldown_enable(bool);
    void set_output_disable(bool);
    void set_input_enable(bool);

    void set_ctrl(uint32_t);
    uint32_t get_ctrl() const;

    void set_status(uint32_t);
    uint32_t get_status() const;

    bool digital_read() const { if (is_connected()) return connected_net()->digital_read(); return false; }

  protected:
    virtual void net_state_changed() override;
    void pad_state_changed();
  private:
    // bool m_input_value_raw;
    // bool m_output_value_raw;
    // bool m_output_disable_raw;
    // bool m_input_enable_raw;
    DriveStrength m_drive_strength;
    GPIO &m_gpio;
  };

}