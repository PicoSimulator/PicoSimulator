#include "clock.hpp"
#include <cassert>
#include <algorithm>

void Clock::sink_add(IClockable &c) {
  m_clock_sinks.push_back(c);
}

void Clock::sink_remove(IClockable &c) {
  m_clock_sinks.erase(std::remove_if(m_clock_sinks.begin(), m_clock_sinks.end(), [&c](IClockable &e) {
    return &e == &c;
  }), m_clock_sinks.end());
}

void Clock::do_tick() {
  for (auto e : m_clock_sinks) {
    e.get().tick();
  }
}

void ClockMux::tick() {
  do_tick();
}

void ClockMux::set_source_index(uint8_t i) {
  assert(i < m_clock_sources.size());
  assert(m_current_source < m_clock_sources.size());
  m_clock_sources[m_current_source]->sink_remove(*this);
  m_current_source = i;
  m_clock_sources[m_current_source]->sink_add(*this);
}

void ClockDiv::tick() {
  if(m_cnt-- == 0) {
    m_cnt = m_divisor;
    do_tick();
  }
}