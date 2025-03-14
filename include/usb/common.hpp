#pragma once

#include <cstdint>

namespace USB {

  uint8_t calc_crc5(const uint8_t *data, uint16_t len_bits);
  uint16_t calc_crc16(const uint8_t *data, uint16_t len_bits);

  /**
   * Append CRC5 to data
   * Should not include PID
   */
  void crc5_append(uint8_t *data, uint16_t len_bits);
  void crc16_append(uint8_t *data, uint16_t len_bits);
}