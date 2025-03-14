#pragma once
#include <cstdint>
#include <vector>

namespace USB::USBIP {
  class Server final {
  public: 
    Server();
    void listen();
      
  protected:
  private:
    int m_server_socket;
    uint16_t m_port;
    std::vector<int> m_client_sockets;
  };
}