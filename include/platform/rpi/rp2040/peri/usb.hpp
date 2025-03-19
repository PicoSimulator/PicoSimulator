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
      {
        out = m_addr | (m_endp << 16);
      } break;
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
    // Device: connected
    void connected();
    // Device: Setup packet received
    void setup_received()
    {
      m_sie_status |= (1<<17);
      m_irqs.get(IRQ::SETUP_REQ).raise();
    }
    /*
    * Transaction complete
    * Raised by device if:
    *   An IN or OUT packet is sent with the LAST_BUFF bit set in buffer ctrl
    * Raised by host if:
    *   A Setup packet is sent when no data IN or data OUT phase follows
    *   An IN packet is received and the LAST_BUFF bit is set
    *   An IN packet is received with zero length
    *   An OUt packet is sent and the LAST_BUFF bit is set
    */ 
    void transaction_complete();
    // Device: bus reset received
    void bus_reset();
    // CRC Error. Raised by the Serial RX engine.
    void crc_error();
    // Bit Stuff Error. Raised by the Serial RX engine.
    void bitstuff_error();
    // RX overflow is raised by the Serial RX engine if the incoming data is too fast.
    void rx_overflow();
    // RX timeout is raised by both the host and device if an ACK is not received in the maximum time specified by the USB spec.
    void rx_timeout();
    // ACK received. Raised by both host and device.
    void ack_received();
    RP2040::RP2040 &m_rp2040;
    uint8_t m_dpsram[0x1000];
    enum IRQ {
      HOST_CONN_DIS        = 0,
      HOST_RESUME          = 1,
      HOST_SOF             = 2,
      TRANS_COMPLETE       = 3,
      BUFF_STATUS          = 4,
      ERROR_DATA_SEQ       = 5,
      ERROR_RX_TIMEOUT     = 6,
      ERROR_RX_OVERFLOW    = 7,
      ERROR_BIT_STUFF      = 8,
      ERROR_CRC            = 9,
      STALL                = 10,
      VBUS_DETECT          = 11,
      BUS_RESET            = 12,
      DEV_CONN_DIS         = 13,
      DEV_SUSPEND          = 14,
      DEV_RESUME_FROM_HOST = 15,
      SETUP_REQ            = 16,
      DEV_SOF              = 17,
      ABORT_DONE           = 18,
      EP_STALL_NAK         = 19,
    };
    InterruptSourceMulti m_irqs;

    uint32_t m_sie_status;

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

    class PHY final : public USB::Transceiver{
    public:
      PHY(RP2040::USB::USB &controller)
      : m_controller{controller} {}
      void receive_packet(const uint8_t *data, uint16_t len_bits) override
      {
        // check CRC
        using ::USB::PID;
        PID pid = static_cast<::USB::PID>(data[0]);
        switch (m_state) {
        case State::EXPECT_TOKEN:
        {
          switch (pid) {
          case PID::SETUP:
          case PID::IN:
          case PID::OUT:
          }
        } break;
        case State::SETUP_DATA:
        {
          if (m_token_addr == m_controller.m_addr && m_token_endp == 0) {
            memcpy(&m_controller.m_dpsram[0], &data[1], len_bits/8-3);
            m_controller.setup_received();
          }
        } break;
        case 
        }
      }

    private:

      RP2040::USB::USB &m_controller;
      uint8_t m_token_addr;
      uint8_t m_token_endp;
      ::USB::PID m_token_pid;
      enum class State {
        EXPECT_TOKEN, // expect one of Setup, In, Out
        SETUP_DATA,
        EXPECT_SETUP_ACK,
        IN_DATA, // expect DATAx/STALL/NAK
        SEND_ACK,
        OUT_DATA,
        OUT_ACK, //expect ACK/NAK/STALL

      } m_state;
    } m_usb_phy;

    GPIOSignal m_dp, m_dm;

  };

}