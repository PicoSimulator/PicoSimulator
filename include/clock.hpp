#pragma once

#include <cstdint>
#include <list>
#include <vector>

class IClockable {
public:
virtual void tick() = 0;
protected:
private:
};

class Clock {
public:
  void sink_add(IClockable *c);
  void sink_remove(IClockable *c);
protected:
  void do_tick();
private:
  std::vector<IClockable*> m_clock_sinks;
};

class ClockTransform : public Clock, public IClockable{};

class ClockMux final : public ClockTransform {
public:
  void tick();
  void set_source_index(uint8_t i);
protected:
private:
  uint8_t m_current_source;
  std::vector<Clock*> m_clock_sources;
};

class ClockDiv final : public ClockTransform {
public:
  void tick();
protected:
private:
  uint32_t m_divisor;
  uint32_t m_cnt;
};

class ClockPLL : public ClockTransform {
public:

};