#pragma once

#include "tracing/vcd.hpp"

namespace Tracing{
  struct WireState{
    enum State{
      LOW,
      HIGH,
      Z,
      X,
    };
    WireState(bool state=false) : m_state{state?HIGH:LOW} {}
    WireState(State state) : m_state{state} {}
    State m_state;
  };
  class Wire : public VCD::Variable{
  public:
    Wire(const std::string &name, bool state=false) 
    : VCD::Variable{name, VCD::Variable::Type::WIRE, 1}
    , m_state{state} {}
    operator bool() const { return m_state; }
    void operator=(bool state) { 
      bool updated = m_state != state;
      m_state = state; 
      if(updated)
        this->updated();
    }
    void set_state(WireState::State state) { 
      bool updated = m_state != state;
      m_state = state; 
      if(updated)
        this->updated();
    }
    bool operator|= (bool state) { return m_state |= state; }
  protected:
    std::ostream &print_state(std::ostream &os) const override {
      os << (m_state ? '1' : '0') << get_id() << std::endl;
      return os;
    }
  private:
    bool m_state;
  };
  class MultiWire : public VCD::Variable{
  public:
    MultiWire(const std::string &name, uint32_t nbits, uint32_t state=0) 
    : VCD::Variable{name, VCD::Variable::Type::WIRE, nbits}
    , m_state{state} {}
    operator uint32_t() const { return m_state; }
    void operator=(uint32_t state) { 
      bool updated = m_state != state;
      m_state = state; 
      // if(updated)
      //   this->updated();
    }
  protected:
    std::ostream &print_state(std::ostream &os) const override {
      os << "b" << (m_state ? '1' : '0') << " " << get_id() << std::endl;
      return os;
    }
  private:
    uint32_t m_state;
  };
};