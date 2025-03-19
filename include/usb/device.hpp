#pragma once

#include "usb/transceiver.hpp"
#include "usb/pid.hpp"
#include <cstdint>

namespace USB {
  class Host;
  class Device : public Transceiver {
    /**
     * Send USB Data packet
     *   Higher level helper function to construct and send a data packet.
     */
    void send_data_packet(DataPID pid, const uint8_t *data, uint16_t len);
    void send_handshake_packet(HandshakePID pid);
  private:
    Host *m_host;
  };

}