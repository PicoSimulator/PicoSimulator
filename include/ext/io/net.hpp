#pragma once

#include <vector>
#include <cstdint>
#include <string>

#include "tracing/wire.hpp"

class Net;

class NetConnection{
public:
  bool is_connected() const;
  Net *connected_net() { return m_connected_net; }
  const Net *connected_net() const { return m_connected_net; }
  uint8_t get_drive_strength() const;
  bool get_drive_value() const;
  virtual void net_state_changed(){}
  void connect_to_net(Net *net);
protected:
  void set_drive_value(bool value){ m_drive_value = value; }
  void set_drive_strength(uint8_t strength){ m_drive_strength = strength; }
private:
  Net *m_connected_net;
  bool m_drive_value;
  // drive strength is measured logarithmically
  // 0 is the weakest, essentially unconnected (100Mohm)
  // 127 is the default (10Kohm)
  // 255 is the strongest (1ohm)
  uint8_t m_drive_strength;
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
