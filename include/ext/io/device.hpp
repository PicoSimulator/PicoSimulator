#pragma once
#include "net.hpp"
#include <map>
#include <vector>
#include <memory>
#include <cassert>

class IODevice {
public:
  IODevice() = default;
  virtual ~IODevice() = default;
  NetConnection *get_named_pin(const std::string &name, bool required=true)
  {
    auto it = m_named_pins.find(name);
    if (it != m_named_pins.end())
      return &it->second;
    for (auto &dev : m_sub_devices)
    {
      auto pin = dev->get_named_pin(name, false);
      if (pin)
        return pin;
    }
    assert(!required);
    return nullptr;
  }
  void add_sub_device(std::unique_ptr<IODevice> &&dev) { m_sub_devices.push_back(std::move(dev)); }
  void add_named_pin(const std::string &name, NetConnection &net) { m_named_pins.emplace(name, net); }
  void rename_pin(const std::string &old_name, const std::string &new_name)
  {
    auto it = m_named_pins.find(old_name);
    assert(it != m_named_pins.end());
    m_named_pins.emplace(new_name, it->second);
    m_named_pins.erase(it);
  }
  void remove_pin(const std::string &name)
  {
    auto it = m_named_pins.find(name);
    assert(it != m_named_pins.end());
    m_named_pins.erase(it);
  }
protected:
private:
  std::map<std::string, NetConnection&> m_named_pins;
  std::vector<std::unique_ptr<IODevice>> m_sub_devices;
};