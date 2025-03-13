#pragma once

#include "usb/pid.hpp"

namespace USB {
  class Device;
  class Host {
  public:
    virtual void receive_packet(const uint8_t *data, uint16_t len_bits);
  protected:
    void send_sof(uint16_t frame_num);
    void send_token_packet(TokenPID pid, uint8_t addr, uint8_t endp);
    void send_data_packet(DataPid pid, const uint8_t *data, uint16_t len);
    void send_handshake_packet(HandshakePID pid);

    virtual void on_receive_error(const uint8_t *data, uint16_t len_bits);
    virtual void on_receive_sof(uint16_t frame_num);
    virtual void on_receive_data_packet(DataPid pid, const uint8_t *data, uint16_t len);
  private:
    Device *m_device;
  };

}