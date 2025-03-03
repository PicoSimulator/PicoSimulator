#pragma once

#include <memory>
#include <span>
#include "ext/io/device.hpp"

extern "C" {
  std::unique_ptr<IODevice> create_device(const std::string &name);
  const std::span<const std::string> device_names();
}