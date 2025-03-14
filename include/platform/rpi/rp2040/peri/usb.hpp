#pragma once

#include "platform/rpi/rp2040/peripheral.hpp"
#include "common/gpio.hpp"

namespace RP2040{
  class RP2040;
}

namespace RP2040::USB{
  
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
  
  class USB final : public IPeripheralPort{
  public:
    USB(RP2040::RP2040 &rp2040) : m_rp2040{rp2040} {}
  protected:
    virtual PortState read_word(uint32_t addr, uint32_t &out) override final
    {
      out = 0;
      if (!(addr & 0x10000)) {
        out = *(uint32_t*)&m_dpsram[addr & 0xfffc];
        return;
      }
      switch(addr & 0xff) {
      case 0x00: // ADDR_ENDP
      case 0x40: // MAIN_CTRL
      case 0x44: // SOF_WR
      case 0x48: // SOF_RD
      case 0x4c: // SIE_CTRL
      case 0x50: // SIE_STATUS
      case 0x54: // INT_EP_CTRL
      case 0x58: // BUFF_STATUS
      case 0x5c: // BUFF_CPU_SHOULD_HANDLE
      case 0x60: // EP_ABORT
      case 0x64: // EP_ABORT_DONE
      case 0x68: // EP_STALL_ARM
      case 0x6c: // NAK_POLL
      case 0x70: // EP_STATUS_STALL_NAK
      case 0x74: // USB_MUXING
      case 0x78: // USB_PWR
      case 0x7c: // USBPHY_DIRECT
      case 0x80: // USBPHY_DIRECT_OVERRIDE
      case 0x84: // USBPHY_TRIM
      case 0x8c: // INTR
      case 0x90: // INTE
      case 0x94: // INTF
      case 0x98: // INTS
      }
    }
    virtual PortState write_word(uint32_t addr, uint32_t in) override final
    {

      switch(addr & 0xff) {
        case 0x00: // ADDR_ENDP
        {
          m_addr = in&0x7f;
          m_endp = (in>>16)&0x0f;
        } break;
        case 0x40: // MAIN_CTRL
        {
          m_sim_timing = in&0x8000;
          m_host_ndevice = in&0x0002;
          m_enabled = in&0x0001;
        } break;
        case 0x44: // SOF_WR
        case 0x48: // SOF_RD
        case 0x4c: // SIE_CTRL
        case 0x50: // SIE_STATUS
        case 0x54: // INT_EP_CTRL
        case 0x58: // BUFF_STATUS
        case 0x5c: // BUFF_CPU_SHOULD_HANDLE
        case 0x60: // EP_ABORT
        case 0x64: // EP_ABORT_DONE
        case 0x68: // EP_STALL_ARM
        case 0x6c: // NAK_POLL
        case 0x70: // EP_STATUS_STALL_NAK
        case 0x74: // USB_MUXING
        case 0x78: // USB_PWR
        case 0x7c: // USBPHY_DIRECT
        case 0x80: // USBPHY_DIRECT_OVERRIDE
        case 0x84: // USBPHY_TRIM
        case 0x8c: // INTR
        case 0x90: // INTE
        case 0x94: // INTF
        case 0x98: // INTS
        }
    }
  private:
    RP2040::RP2040 &m_rp2040;
    uint8_t m_dpsram[0x1000];

    // ADDR_ENDP
    uint8_t m_addr;
    uint8_t m_endp;
    // MAIN_CTRL
    bool m_sim_timing;
    bool m_host_ndevice;
    bool m_enabled;
    // SOF
    uint16_t m_next_sof;
    uint16_t m_sof;
    // SIE
    bool m_resume;
  };

}