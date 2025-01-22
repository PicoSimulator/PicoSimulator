#pragma once

#include <cstdint>

class GDBServer{
public:
  virtual void startServer(uint16_t port) = 0;
protected:
private:
};