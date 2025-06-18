#pragma once

#include <vector>
#include <cstdint>
#include <string>

#include "tracing/wire.hpp"

class Net;
using drive_strength_t = uint8_t;
class DriveStrength{
public:
  DriveStrength impede(unsigned long resistance) {
    __builtin_unreachable();
    // Placeholder for future implementation.
  }
protected:
private:
};

class NetConnection{
public:
  NetConnection() = default;
  bool is_connected() const;
  Net *connected_net() { return m_connected_net; }
  const Net *connected_net() const { return m_connected_net; }
  uint8_t get_drive_strength() const;
  bool get_drive_value() const;
  virtual void net_state_changed(){}
  void connect_to_net(Net *net);
  void set_drive_value(bool value){ m_drive_value = value; }
  void set_drive_strength(uint8_t strength){ m_drive_strength = strength; }
  static NetConnection *special(const std::string &name);
protected:
  NetConnection(bool special_net) : m_special_net{special_net} {}
private:
  Net *m_connected_net = nullptr;
  bool m_drive_value = 0;
  // drive strength is measured logarithmically
  // 0 is 100Mohm, treated as high impedance/floating
  // 127 is the default (10Kohm)
  // 159 1Kohm
  // 191 100ohm
  // 223 10ohm
  // 255 is the strongest (1ohm)
  // 
  drive_strength_t m_drive_strength = 0;
  // metadata allows us to optimise certain device interfaces
  // e.g. SPI devices can be optimised if the correct pins are connected
  // if we see SCLK, check if MOSI or MISO are connected
  bool m_special_net = false;
};

class Net final{
  friend class NetConnection;
public:
  Net(const std::string &name);
  const std::string &name() const { return m_name; }
  bool digital_read() const { return m_state; }
  float analog_read() const { return m_state?3.3:0; }
  void update();
  Tracing::VCD::Variable &vcd_variable() { return m_state; }
protected:
  void add_connection(NetConnection *conn);
  void remove_connection(NetConnection *conn);
private:
  const std::string m_name;
  // metadata allows us to optimise certain device interfaces
  std::vector<NetConnection*> m_connections;
  Tracing::Wire m_state;
};
