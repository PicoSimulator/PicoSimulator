#include "ext/io/net.hpp"

#include <iostream>

Net::Net(const std::string &name)
  : m_name{name}
  , m_state{name, false}
{}

void Net::update(){
  bool old_state = m_state;
  bool new_state = m_state;
  uint8_t drive_strength = 0;
  for (auto &c : m_connections){
    if(c->get_drive_strength() <= drive_strength)
      continue;
    drive_strength = c->get_drive_strength();
    new_state = c->get_drive_value();
  }
  if (old_state != new_state) {
    m_state = new_state;
    // std::cerr << "Net " << m_name << " changed to " << m_state << std::endl;
    for(auto &c : m_connections){
      c->net_state_changed();
    }
  }
  if (drive_strength == 0) {
    m_state.set_state(Tracing::WireState::Z);
  }
}

void Net::add_connection(NetConnection *conn){
  m_connections.push_back(conn);
  update();
}

void Net::remove_connection(NetConnection *conn){
  for(auto it = m_connections.begin(); it != m_connections.end(); it++){
    if(*it == conn){
      m_connections.erase(it);
      return;
    }
  }
  update();
}

void NetConnection::connect_to_net(Net *net){
  if (this == nullptr)
    return;
  if (m_connected_net && !m_special_net)
    m_connected_net->remove_connection(this);
  m_connected_net = net;
  m_connected_net->add_connection(this);
}

bool NetConnection::is_connected() const{
  return m_connected_net != nullptr;
}


uint8_t NetConnection::get_drive_strength() const{
  return m_drive_strength;
}

bool NetConnection::get_drive_value() const{
  return m_drive_value;
}

NetConnection *NetConnection::special(const std::string &name){
  static NetConnection gnd{true};
  gnd.set_drive_strength(255);
  gnd.set_drive_value(0);
  static NetConnection vcc{true};
  vcc.set_drive_strength(255);
  vcc.set_drive_value(1);
  if(name == "GND")
    return &gnd;
  if(name == "VCC")
    return &vcc;
  return nullptr;
}