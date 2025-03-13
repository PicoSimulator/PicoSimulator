#pragma once

#include <cstddef>
#include <cstdint>
#include <queue>
#include <limits>
#include <map>
#include <memory>
#include "ext/io/device.hpp"
#include "common/callable.hpp"
#include "clock.hpp"

class Simulation;
typedef uint64_t simulation_time_t;

#include "tracing/vcd.hpp"

#define TICK_FREQ_GHZ 10

class Simulation final{
public:
  typedef std::size_t tick_t;
  // 100ps per tick
  static Simulation &get();
  static simulation_time_t long_wait = 


  void run(simulation_time_t max_time) {
    for (auto &[name, device] : m_components) {
      device->ready();
    }
    if (m_schedule.empty()) {
      std::cerr << "Nothing to simulate!\n";
      return;
    }
    while(m_time < max_time) {
      auto &[time, c, period] = m_schedule.top();
      m_schedule.pop();
      m_time = time;
      c();
      if(period)
        schedule_periodic(period, c.get());
    }
  }
  void abort() {
    m_time = std::numeric_limits<simulation_time_t>::max();
  }
  void exit() {
    m_components.clear();
  }
  simulation_time_t current_time() const { return m_time; }
  void schedule(simulation_time_t time, ICallable &c) {
    m_schedule.push({time, c, 0});
  }
  void schedule_in(simulation_time_t time, ICallable &c) {
    m_schedule.push({m_time + time, c, 0});
  }
  void schedule_periodic(simulation_time_t period, ICallable &c)
  {
    m_schedule.push({m_time + period, c, period});
  }
  constexpr static unsigned to_seconds(simulation_time_t t) { return t / from_seconds(1); }
  constexpr static unsigned to_milliseconds(simulation_time_t t) { return t / from_milliseconds(1); }
  constexpr static unsigned to_microseconds(simulation_time_t t) { return t / from_microseconds(1); }
  constexpr static unsigned to_nanoseconds(simulation_time_t t) { return t / from_nanoseconds(1); }
  constexpr static simulation_time_t from_seconds(unsigned s) { return s*from_milliseconds(1'000); }
  constexpr static simulation_time_t from_milliseconds(unsigned ms) { return ms*from_microseconds(1'000); }
  constexpr static simulation_time_t from_microseconds(unsigned us) { return us*from_nanoseconds(1'000); }
  constexpr static simulation_time_t from_nanoseconds(unsigned ns) { return ns*TICK_FREQ_GHZ; }
  constexpr static simulation_time_t from_hz(unsigned hz) { return from_seconds(1) / hz; }

  Tracing::VCD::VCDFile &vcd() { return m_vcd; }
  Tracing::VCD::Module &nets() { return m_nets; }

  auto &components() { return m_components; }

protected:
private:
  Simulation()
  {
    m_vcd.top().add_item(m_nets);
    m_vcd.top().add_item(m_time);
  }
  struct Event{
    simulation_time_t time;
    std::reference_wrapper<ICallable> callable;
    simulation_time_t period;
    bool operator<(const Event &rhs) const {
      return time < rhs.time;
    }
  };
  std::priority_queue<
    Event,
    std::deque<Event>
    // std::vector<Event>
  > m_schedule;
  // simulation_time_t m_time;
  Tracing::VCD::Module m_nets{"NETS"};
  Tracing::VCD::Time<simulation_time_t> m_time{"Simulation_Time", 64};
  Tracing::VCD::VCDFile m_vcd;
  std::map<std::string, std::unique_ptr<IODevice>> m_components;
};