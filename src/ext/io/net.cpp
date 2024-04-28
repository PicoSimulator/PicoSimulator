#include "ext/io/net.hpp"

void Net::update(){
  uint8_t drive_strength = 0;
  for (auto &c : m_connections){
    if(c->get_drive_strength() <= drive_strength)
      continue;
    drive_strength = c->get_drive_strength();
    m_state = c->get_drive_value();
  }
 
  for(auto &c : m_connections){
    c->net_state_changed();
  }
}

void Net::add_connection(NetConnection *conn){
  m_connections.push_back(conn);
}

void Net::remove_connection(NetConnection *conn){
  for(auto it = m_connections.begin(); it != m_connections.end(); it++){
    if(*it == conn){
      m_connections.erase(it);
      return;
    }
  }
}

void NetConnection::connect_to_net(Net *net){
  if (m_connected_net)
    m_connected_net->remove_connection(this);
  m_connected_net = net;
  m_connected_net->add_connection(this);
}

bool NetConnection::is_connected() const{
  return m_connected_net != nullptr;
}

Net *NetConnection::connected_net(){
  return m_connected_net;
}

uint8_t NetConnection::get_drive_strength() const{
  return m_drive_strength;
}

bool NetConnection::get_drive_value() const{
  return m_drive_value;
}

