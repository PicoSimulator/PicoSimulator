#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"

namespace RP2040{
  class RP2040;
}

enum class PacketType : uint8_t{
  SETUP = 0,
  DATA = 1,
  ACK = 2,
  NAK = 3,
  STALL = 4,
  NYET = 6,
  PRE = 7
};

enum class PID : uint8_t{
  OUT   = 0b0001,
  IN    = 0b1001,
  SOF   = 0b0101,
  SETUP = 0b1101,
  DATA0 = 0b0011,
  DATA1 = 0b1011,
  DATA2 = 0b0111,
  MDATA = 0b1111,
  ACK   = 0b0010,
  NAK   = 0b1010,
  STALL = 0b1110,
  NYET  = 0b0110,
  PRE   = 0b1100
};

class USBInterface{
public:
  virtual ~USBInterface() = default;
  virtual void receive_packet(uint8_t *data, size_t len) = 0;
protected:
private:
};

class USB final : public IPeripheralPort{
public:
  USB(RP2040::RP2040 &rp2040) : m_rp2040{rp2040} {}
protected:
private:
  RP2040::RP2040 &m_rp2040;
  uint8_t m_dpsram[0x1000];
};