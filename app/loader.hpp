#pragma once

#include "ext/io/device.hpp"
#include <memory>

std::unique_ptr<IODevice> create_device(const std::string &name);