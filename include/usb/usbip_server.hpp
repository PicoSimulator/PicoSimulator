#pragma once

namespace USB::USBIP {
  class Server final {
  public: 
    Server();
    void listen();
      
  protected:
  private:
    int m_server_socket;
    std::vector<int> m_client_sockets;
  };
}