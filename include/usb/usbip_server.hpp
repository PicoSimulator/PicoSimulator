#pragma once
#include <cstdint>
#include <vector>

#include <map>

namespace USB::USBIP {
  class Server final {
  public: 
    Server();
    void listen();
      
  protected:
  private:
    int m_server_socket;
    struct SocketData{
      bool m_has_device;

    };
    std::map<int, SocketData> m_client_sockets;
    uint16_t m_port;
  };
#pragma pack(push)
  struct OP_REQ_Header {
    uint16_t ver;
    uint16_t cmd;
    uint32_t status;
  };
static_assert(sizeof(REQ_Header) == 8);
  struct OP_REP_DEVLIST {
    OP_REQ_Header header;
  };
  struct OP_REQ_IMPORT_Packet {
    char busid[32];
  };
  struct USBIP_CMD_Header_Basic {
    uint32_t commad;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
  };
static_assert(sizeof(USBIP_CMD_Header_Basic) == 20);
  struct USBIP_CMD_SUBMIT_Packet {
    uint32_t tranfer_flags;
    uint32_t transfer_buffer_length;
    uint32_t start_frame;
    uint32_t num_iso_packets;
    uint32_t interval;
    uint8_t[8] setup;
  };
#pragma pack(pop)
}