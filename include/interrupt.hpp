#pragma once

#include <string>
#include <tuple>
#include <array>

#define ENUM_IRQ_TYPE(o) \
  o(raise_latched) \
  o(raise) \
  o(lower)

#define ENUM_32(o) \
  o(0) \
  o(1) \
  o(2) \
  o(3) \
  o(4) \
  o(5) \
  o(6) \
  o(7) \
  o(8) \
  o(9) \
  o(10) \
  o(11) \
  o(12) \
  o(13) \
  o(14) \
  o(15) \
  o(16) \
  o(17) \
  o(18) \
  o(19) \
  o(20) \
  o(21) \
  o(22) \
  o(23) \
  o(24) \
  o(25) \
  o(26) \
  o(27) \
  o(28) \
  o(29) \
  o(30) \
  o(31)

class InterruptSink {
public:
  virtual void update() = 0;
protected:
private:
};

class InterruptSource{
public:
  InterruptSource(InterruptSink &sink, std::bitset<32>::reference raised_bit) : m_raised_bit(raised_bit), m_sink(sink) {}
  void raise()
  {
    m_raised_bit = true;
    m_sink.update();
  }
  void lower()
  {
    m_raised_bit = false;
    m_sink.update();
  }
  void raise_latched()
  {
    m_raised_bit = true;
    m_sink.update();
  }
  void set_name(const std::string &name)
  {
    m_name = name;
  }
  operator bool() const
  {
    return m_raised_bit;
  }
protected:
private:
  std::string m_name;
  std::bitset<32>::reference m_raised_bit;
  InterruptSink &m_sink;
};


class InterruptSourceMulti final : private InterruptSink{
public:
#define EVAL_SOURCES(n) {*this, m_raw_irq_status[n]},
  InterruptSourceMulti(InterruptSource &source) 
  : m_source(source) 
  , m_sources{{
    ENUM_32(EVAL_SOURCES)
  }}
  {}
#undef EVAL_SOURCES

  InterruptSource &get_source(int irq_num)
  {
    return m_sources[irq_num];
  }

  InterruptSource &operator[](int irq_num)
  {
    return get_source(irq_num);
  }

  void mask(uint32_t mask)
  {
    m_irq_mask = mask;
    update();
  }
  uint32_t mask() const 
  {
    return m_irq_mask.to_ulong();
  }
  uint32_t masked() const {
    return m_masked_irq_status.to_ulong();
  }

  uint32_t raw() const
  {
    return m_raw_irq_status.to_ulong();
  }

  void force(uint32_t force)
  {
    m_masked_irq_status |= force;
    check();
  }

protected:
private:
  virtual void update() override final {
    m_masked_irq_status = m_raw_irq_status & m_irq_mask;
    check();
  }
  void check() {
    if (m_masked_irq_status.any())
      m_source.raise();
    else 
      m_source.lower();
  }
  std::bitset<32> m_raw_irq_status;
  std::bitset<32> m_masked_irq_status;
  std::bitset<32> m_irq_mask;
  InterruptSource &m_source;
  std::array<InterruptSource, 32> m_sources;
};


class InterruptSourceSet final : private InterruptSink{
public:
#define EVAL_SOURCES(n) {*this, m_raised[n]},
  InterruptSourceSet()
  : m_sources{{
    ENUM_32(EVAL_SOURCES)
  }}{}
#undef EVAL_SOURCES

  InterruptSource &get_source(int irq_num)
  {
    return m_sources[irq_num];
  }

  uint32_t raised() const
  {
    return m_raised.to_ulong();
  }

  void set_pending(uint32_t pending)
  {
    m_raised |= pending;
  }
  void clear_pending(uint32_t pending)
  {
    m_raised &= ~pending;
  }

protected:
private:
  virtual void update() override final
  {}
  std::bitset<32> m_raised;
  std::array<InterruptSource, 32> m_sources;
};
