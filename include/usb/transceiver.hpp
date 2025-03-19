#pragma once

namespace USB{

  class Transceiver {
  public:
    virtual void receive_packet(const uint8_t *data, uint16_t len_bits) = 0;
  protected:
    /**
      * Send raw USB packet
      * Excludes: SYNC,EOP
      * INCLUDES: PID[,data,CRC]
      */
    void send_packet(const uint8_t *data, uint16_t len_bits);
  };

}