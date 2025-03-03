#include "platform/rpi/rp2040/bus/ahb_lite.hpp"

using namespace RP2040::Bus;

void AHBLite::tick() {
  m_runner.tick();
}

ClockTask AHBLite::bus_task() {
  while (true) {
    co_await ClockTask::next_tick();
  }
}

PortState AHBLite::read_word_internal(uint32_t addr, uint32_t &out) {
  switch(addr & 0x00f0'0000) {
    case 0x0000'0000: return m_dma.read_word(addr, out); break;
    case 0x0010'0000: out = 0x0000'0000; break;
    case 0x0020'0000: out = 0x0000'0000; break;
    case 0x0030'0000: out = 0x0000'0000; break;
    case 0x0040'0000: out = 0x0000'0000; break;
  }
  std::cout << "AHBLite::read_word_internal(" << std::hex << addr << ")" << std::endl;
  return PortState::SUCCESS;
}

PortState AHBLite::write_word_internal(uint32_t addr, uint32_t in) {
  switch(addr & 0x00f0'0000) {
    case 0x0000'0000: return m_dma.write_word(addr, in); break;
    case 0x0010'0000: ; break;
    case 0x0020'0000: ; break;
    case 0x0030'0000: ; break;
    case 0x0040'0000: ; break;
  }
  std::cout << "AHBLite::write_word_internal(" << std::hex << addr << ", " << in << ")" << std::endl;
  return PortState::SUCCESS;
}

uint32_t AHBLite::read_word_internal_pure(uint32_t addr) const {
  return 0;
}