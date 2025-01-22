#include "debug/rp2040_gdb.hpp"

void RP2040GDBServer::startServer(uint16_t port)
{
  // Start the server
  (void)port;
}

void RP2040GDBServer::stopServer()
{
  // Stop the server
}

void RP2040GDBServer::sendPacket(const std::span<std::byte> packet)
{
  // Send a packet
  (void)packet;
}

void RP2040GDBServer::runThread()
{
  // Run the server thread
  while (true)
  {
    // Do something
  }
}