#pragma once

#include <exception>
#include <string>
// #include <format>

namespace ARMv6M{
  class HardFault : public std::exception{
  public:
    HardFault(): m_msg{"Generic HardFault"}{}
    HardFault(std::string msg) : m_msg{msg}{}
    const char* what() const noexcept override { return m_msg.c_str(); }
    std::string m_msg;
  };

  class BusFault : public HardFault{
  public:
    BusFault() : HardFault{"BusFault -> HardFault"}{}
    // BusFault(uint32_t addr) : HardFault{std::format("BusFault({:x}) -> HardFault" ,addr)}{}
    BusFault(uint32_t addr) : HardFault{"BusFault(" + std::to_string(addr)+ ") -> HardFault"}{}
  protected:
  private:
  };

  class UnimplementedFault : public HardFault{
  public:
    UnimplementedFault() = delete;
    UnimplementedFault(std::string && msg) : HardFault{std::move(msg)} {}
  };
}
