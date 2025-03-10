#pragma once

#include "net.hpp"
#include "device.hpp"
#include <string>

class Resistor final : public IODevice {
public:
  Resistor()
  : IODevice()
  , m_pins{
    {*this},
    {*this}
  }
  , m_resistance{1000}
  {
    add_named_pin("A", m_pins[0]);
    add_named_pin("B", m_pins[1]);
  }
  void set_resistance(unsigned resistance) {
    m_resistance = resistance;

  }

  bool set_param(const std::string &name, const std::string &value) override {
    if (name == "resistance" || name == "value") {
      char lastChar = std::tolower(value.back());
      unsigned multiplier = 1;
      if (lastChar > '9' || lastChar < '0') {
        if (lastChar == 'k') {
          multiplier = 1000;
        } else if (lastChar == 'm') {
          multiplier = 1000000;
        } else {
          return false;
        }
      }

      set_resistance(std::stoul(value) * multiplier);
      return true;
    }
    return false;
  }
protected:
  void propogate() {
    if (!m_pins[0].is_connected()) return;
    if (!m_pins[1].is_connected()) return;
    auto &net0 = *m_pins[0].connected_net();
    auto &net1 = *m_pins[1].connected_net();
    // assume impedance 64 (1M ohm)
    // assume drive impedance 32 (10Mohm)
    // 32/64 = 0. Minimal effect on impedance (subtract 1)
    // 32/32 = 1. Impedance is halved (subtract 10 -> 22)
    // 64/32 = 2. Impedance is 10 fold (subtract 33 -> 31)
    // 128/64 = 2. Impedance is 100 fold (subtract 65 -> 63)
    // 128/32 = 4. Impedance is 1000 fold (subtract 97)
    // multiply by 
    auto combine_drive = [](unsigned a, unsigned b) {
      return std::max(1u, std::min(a, b))-1;
    };
    auto drv0 = m_pins[0].get_drive_strength();
    auto drv1 = m_pins[1].get_drive_strength();
    auto val0 = m_pins[0].get_drive_value();
    auto val1 = m_pins[1].get_drive_value();
    {
      m_pins[0].set_drive_strength(combine_drive(drv1, m_drive_impedance));
      m_pins[0].set_drive_value(val1);
    }
    {
      m_pins[1].set_drive_strength(combine_drive(drv0, m_drive_impedance));
      m_pins[1].set_drive_value(val0);
    }
  }
private:
  class ResistorPin final : public NetConnection {
  public:
    ResistorPin(Resistor &resistor) 
    : NetConnection{} 
    , m_resistor{resistor}
    {
    }
    void net_state_changed() override {
      m_resistor.propogate();
    }
  private:
    Resistor &m_resistor;
  } m_pins[2];
  unsigned m_resistance;
  drive_strength_t m_drive_impedance;
};