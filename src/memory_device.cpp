#include "memory_device.hpp"

MemoryOperation::awaiter MemoryOperation::return_void() {
  assert(is_write());
  m_port.deregister_op(*this);
  // m_caller.resume();
  return awaiter{m_caller};
}

MemoryOperation::awaiter MemoryOperation::return_value(uint32_t value) {
  assert(is_read());
  m_port.deregister_op(*this);
  data = value;
  // m_caller.resume();
  return awaiter{m_caller};
}

void MemoryOperation::await_suspend(std::coroutine_handle<> h) {
  m_caller = h;
  m_port.register_op(*this);
}