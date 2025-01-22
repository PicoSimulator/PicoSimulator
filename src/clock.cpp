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

void Clock::do_tock() {
  for (auto e : m_clock_sinks) {
    e.get().tock();
  }
}

void ClockMux::tick() {
  do_tick();
}

void ClockMux::tock() {
  do_tock();
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

  // this has huge performance implications
  // I'm not sure what to do with this currently


  return;
  // if m_cnt is zero then we reset the counter
  // if m_cnt is -1 this means we've missed tock
  // tock never decrements the counter
  bool clear = m_cnt == 0;
  
  bool missed = int32_t(m_cnt) < 0;
  if(clear || missed) {
    m_cnt += m_divisor-1;
    // if m_divisor is 0 then next tock should reset the counter
    // if the tock is missed, then we should reset the counter twice
    do_tick_tock();      

    if (missed){
      m_cnt += m_divisor;
      // assert(m_tick_ntock == false);
      do_tick_tock();
      // tick();

    } else {
      // do_tick_tock();
    }
  } else {
    m_cnt -= 2;
  }
}
void ClockDiv::tock() {
  // if m_cnt is -1 then we do reset the counter
  // as this is to be handled by us
  // if m_cnt is 0 then it belongs to tick
  bool clear = int32_t(m_cnt) < 0;
  if (clear) {
    m_cnt = m_divisor;
    do_tick_tock();
  }
}
  
void SimpleClockDiv::tick() {
  if(m_cnt-- == 0) {
    m_cnt = m_divisor;
    do_tick();
  }
}