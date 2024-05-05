#include "memory_device.hpp"

void MemoryOperation::return_void() {
  assert(is_write());
  // DO NOT ENABLE THESE PRINTS THEY ADD 200MB TO THE LOGS PER MCYCLES
  // std::cout << "MemoryOperation::return_void" << std::endl;
  // m_port.deregister_op(*this);
  // m_caller.resume();
  // return awaiter{m_caller};
}

void MemoryOperation::return_value(uint32_t value) {
  assert(is_read());
  // DO NOT ENABLE THESE PRINTS THEY ADD 200MB TO THE LOGS PER MCYCLES
  // std::cout << "MemoryOperation::return_value " << value << std::endl;
  // m_port.deregister_op(*this);
  data = value;
  // m_caller.resume();
  // return awaiter{m_caller};
}

void MemoryOperation::await_suspend(std::coroutine_handle<> h) {
  m_caller = h;
  // m_port.register_op(*this);
}

bool MemoryOperation::await_ready() {
  // attempt to complete the operation immediately
  // by registering the operation and observing the
  // return value
  return m_port.register_op(*this);
}