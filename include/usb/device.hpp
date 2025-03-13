#pragma once

#include "usb/pid.hpp"

namespace USB {
  class Host;
  class Device {
  public:
    virtual void receive_packet(const uint8_t *data, uint16_t len_bits) = 0;
  protected:
    /**
     * Send raw USB packet
     * Excludes: SYNC,EOP
     * INCLUDES: PID[,data,CRC]
     */
    void send_packet(const uint8_t *data, uint16_t len_bits);
    /**
     * Send USB Data packet
     *   Higher level helper function to construct and send a data packet.
     */
    void send_data_packet(DataPid pid, const uint8_t *data, uint16_t len);
    void send_handshake_packet(HandshakePID pid);
  private:
    Host *m_host;
  };

}