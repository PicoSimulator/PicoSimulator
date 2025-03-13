#pragma once

#include <cstdint>
#include <list>
#include <vector>

class IClockable : public ICallable {
public:
// Called on the rising edge of the clock
virtual void tick() = 0;
// Called on the falling edge of the clock
virtual void tock(){};
virtual void operator()() final override { tick(); }
protected:
private:
};

class Clock {
public:
  void sink_add(IClockable &c);
  void sink_remove(IClockable &c);
protected:
  void do_tick();
  void do_tock();
private:
  std::vector<std::reference_wrapper<IClockable>> m_clock_sinks;
  bool m_enabled;
};

class ClockTransform : public Clock, public IClockable{};

class ClockMux final : public ClockTransform {
public:
  void tick() override;
  void tock() override;
  void set_source_index(uint8_t i);
protected:
private:
  uint8_t m_current_source;
  std::vector<Clock*> m_clock_sources;
};

class ClockDiv final : public ClockTransform {
public:
  ClockDiv() : m_divisor{0}, m_cnt{0}, m_tick_ntock{true} {}
  void tick();
  void tock();
  uint32_t divisor() const { return m_divisor; }
  void divisor(uint32_t d) { m_divisor = d-1; }
  void reset() { m_cnt = 0; m_tick_ntock = true; }
  uint32_t count() const { return m_cnt; }
  bool state() const { return !m_tick_ntock; }
protected:
private:
  void do_tick_tock() {
    if (m_tick_ntock) {
      do_tick();
    } else {
      do_tock();
    }
    m_tick_ntock = !m_tick_ntock;
  }
  uint32_t m_divisor;
  uint32_t m_cnt;
  bool m_tick_ntock;
};

class SimpleClockDiv final : public ClockTransform {
public:
  SimpleClockDiv() : m_divisor{0}, m_cnt{0} {}
  void tick();
  uint32_t divisor() const { return m_divisor; }
  void divisor(uint32_t d) { m_divisor = d-1; }
  void reset() { m_cnt = m_divisor; }
  void clear() { m_cnt = 0; }
  uint32_t count() const { return m_cnt; }
protected:
private:
  uint32_t m_divisor;
  uint32_t m_cnt;
};

// template<class ...Ts>
// class ClockTransformChain final : public ClockTransform {
// public:
//   void tick();
  

class ClockPLL : public ClockTransform {
public:
  ClockPLL() 
  : m_vco{*this}
  , m_feedback{*this} {
    sink_add(m_feedback);
  }
  void tick() override;
private:
  IClockable &feedback() { return m_feedback; }
  class PLLVCO : public IClockable {
  public:
    PLLVCO(ClockPLL &pll) : m_pll{pll} {}
    void tick() override {}
    void tock() override {}
  private:
    ClockPLL &m_pll;
  };
  class PLLFeedback : public IClockable {
  public:
    PLLFeedback(ClockPLL &pll) : m_pll{pll} {}
    void tick() override {}
    void tock() override {}
  private:
    ClockPLL &m_pll;
  };
  PLLVCO m_vco;
  PLLFeedback m_feedback;
};