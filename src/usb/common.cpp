#include "usb/common.hpp"
#include <algorithm>


static void crc_append(uint8_t *data, uint16_t len_bits, const uint8_t *crc, uint8_t crc_bits)
{
  data += len_bits/8;
  uint8_t bitpos = len_bits%8;
  uint8_t crc_bitpos = 0;
  do {
    *data &= ((1<<bitpos)-1); // mask lowbits
    *data |= (*crc >> crc_bitpos) << bitpos;
    uint8_t nbits = std::min<uint8_t>(std::min<uint8_t>(8-bitpos, crc_bits), 8-crc_bitpos);
    crc_bitpos += nbits;
    bitpos += nbits;
    data += bitpos/8;
    crc += crc_bitpos/8;
    bitpos %= 8;
    crc_bitpos %= 8;
    crc_bits -= nbits;
  } while(crc_bits > 0);
}

uint16_t USB::calc_crc16(const uint8_t *data, uint16_t len_bits)
{
  uint16_t res = 0x1f;
  uint16_t poly = 0xA001;
  uint8_t b;
  uint8_t bitpos = 0;
  while(len_bits--) {
    b = (*data >> bitpos) ^ res;
    res >>= 1;
    if (b&1) {
      res ^= poly;
    }
    bitpos++;
    data += bitpos/8;
    bitpos %= 8;
  }
  return res ^ 0x1f;
}

uint8_t USB::calc_crc5(const uint8_t *data, uint16_t len_bits)
{
  uint8_t res = 0x1f;
  uint8_t poly = 0x14;
  uint8_t b;
  uint8_t bitpos = 0;
  while(len_bits--) {
    b = (*data >> bitpos) ^ res;
    res >>= 1;
    if (b&1) {
      res ^= poly;
    }
    bitpos++;
    data += bitpos/8;
    bitpos %= 8;
  }
  return res ^ 0x1f;
}

void USB::crc5_append(uint8_t *data, uint16_t len_bits)
{
  uint8_t crc5 = USB::calc_crc5(data, len_bits);
  crc_append(data, len_bits, &crc5, 5);
}

void USB::crc16_append(uint8_t *data, uint16_t len_bits)
{
  uint16_t crc16 = USB::calc_crc16(data, len_bits);
  crc_append(data, len_bits, (uint8_t*)&crc16, 16);
}