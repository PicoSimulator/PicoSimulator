#include "tracing/vcd.hpp"
#include "simulation.hpp"
#include <stdexcept>
#include <format>

using namespace Tracing::VCD;

Item &Item::operator[](const std::string &key) {
  if (get_type() != Type::Module) {
    throw std::runtime_error{"Item must be Module to access children."};
  }
  auto &self = static_cast<Module&>(*this);
  return self.get(key);
}

Item &Module::get(const std::string &key) {
  for (auto &item : m_items) {
    if (item.get().get_name() == key) {
      return item.get();
    }
  }
  throw std::runtime_error(std::format("Child {} does not exist for {}", key, get_name()));
}

void VCDFile::check_time() {
  auto current_time = Simulation::get().current_time();
  if(m_time != current_time){
    *this << "#" << current_time << std::endl;
    m_time = current_time;
  }
}

#if config_VCD_TRACE_ENABLED
void Variable::updated() { 
  if (file()) *file() << *this;
}
#endif

Module &Item::as_module() { return static_cast<Module&>(*this); }
Variable &Item::as_variable() { return static_cast<Variable&>(*this); }
