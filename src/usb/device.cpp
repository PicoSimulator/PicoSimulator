#include "usb/device.hpp"
#include "usb/host.hpp"

using namespace USB;

void Device::send_packet(const uint8_t *data, uint16_t len_bits)
{
  if (m_host) {
    m_host->receive_packet(data, len_bits);
  }
}

void Device::send_data_packet(DataPid pid, const uint8_t *data, uint16_t len)
{
  uint8_t buf[1025];
  uint16_t idx = 0;
  buf[0] = reinterpret_cast<uint8_t>(pid);
  while (len--) {
    buf[idx+1] = data[idx];
    idx++;
  }
  send_packet(buf, 8*(idx+1));
}