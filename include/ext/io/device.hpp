#pragma once
#include "net.hpp"
#include <map>
#include <tuple>
#include <memory>
#include <cassert>
#include "ext/component.hpp"

class IODevice : public Component {
public:
  IODevice() = default;
  virtual ~IODevice() = default;
  NetConnection *get_named_pin(const std::string &name, bool required=true)
  {
    auto it = m_named_pins.find(name);
    if (it != m_named_pins.end())
      return &it->second;
    for (auto &[key, value] : m_sub_devices)
    {
      auto &dev = std::get<0>(value);
      bool available = std::get<1>(value);
      if (!available)
        continue;
      auto pin = dev->get_named_pin(name, false);
      if (pin)
        return pin;
    }
    assert(!required);
    return nullptr;
  }
  void add_sub_device(const std::string &name, std::unique_ptr<IODevice> &&dev, bool make_available = false) { m_sub_devices.emplace(name, std::make_tuple(std::move(dev), make_available)); }
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
  void ready() override
  {
    for (auto &[name, device] : m_sub_devices) {
      device->ready();
    }
  }
protected:
private:
  std::map<std::string, NetConnection&> m_named_pins;
  std::map<std::string, std::tuple<std::unique_ptr<IODevice>, bool>> m_sub_devices;
};