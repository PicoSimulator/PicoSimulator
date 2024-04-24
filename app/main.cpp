#include "rp2040.hpp"

RP2040 g_rp2040{};

int main(int argc, char** argv)
{
  g_rp2040.reset();
  g_rp2040.run();
}