#pragma once

#include "usb/pid.hpp"

namespace USB {
  class Device;
  class Host {
  public:
    void send_sof();
    void send_token_packet(TokenPID pid);
    void send_data_packet()
    void 
  protected:
  private:
    Device *m_device;
  };

}