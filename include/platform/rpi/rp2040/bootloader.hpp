#pragma once

#include <array>
#include <cstdint>

enum class BootloaderVersion{
  B0,
  B1,
  B2,
};

const std::array<uint8_t, 16384> &load_bootloader(BootloaderVersion ver);