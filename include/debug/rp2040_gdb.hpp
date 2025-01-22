#include "debug/gdbserver.hpp"

#include <thread>
#include <span>
#include <queue>

class RP2040GDBServer final : public GDBServer{
public:
  void startServer(uint16_t port) override;
  void stopServer();

protected:
  void sendPacket(const std::span<std::byte> packet);
  void runThread();
private:
  std::thread m_server_thread;
  struct packet{
    std::unique_ptr<std::byte[]> data;
    bool sent;
  };
  std::queue<packet> m_packet_queue;
  std::mutex m_packet_queue_mutex;
  bool m_ack_enabled = true;
};
