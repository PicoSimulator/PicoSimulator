#pragma once

namespace USB {
  class Host;
  class Device {
  public:
    void send_sof();
    void send_token_packet();
    void 
  protected:
  private:
    Host *m_host;
  };

}