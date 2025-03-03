#pragma once
#include "spidev.hpp"

class SPIMaster{

  // list of connected devices
  // if a device has its clock connected and MOSI or MISO
  class SPIConnection {
  public:
  protected:
  private:
    SPIDev &dev;
    /**
     * SCLK (required)
     * MOSI (optional if MISO connected)
     * MISO (optional if MOSI connected)
     * ~SS/~CS (optional, often controlled externally)
     */
  };
};