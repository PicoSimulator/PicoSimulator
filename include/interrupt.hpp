#pragma once

#include <string>
#include <tuple>
#include <vector>

#define ENUM_IRQ_TYPE(o) \
  o(raise_latched) \
  o(raise) \
  o(lower)

class IInterruptSink{
public:
  #define IRQ_TYPE_EVAL(x) virtual void x(int irq_num) = 0;
  ENUM_IRQ_TYPE(IRQ_TYPE_EVAL)
  #undef IRQ_TYPE_EVAL
};

class InterruptSource final{
public:
  InterruptSource(std::string &&name) : m_name(name), m_enabled(false) {}
  #define IRQ_TYPE_EVAL(x) void x()\
  {\
    if (!m_enabled) return;\
    for (auto &cb : m_callbacks)\
    {\
      std::get<0>(cb)->x(std::get<1>(cb));\
    }\
  }
  ENUM_IRQ_TYPE(IRQ_TYPE_EVAL)
  #undef IRQ_TYPE_EVAL
  void connect(IInterruptSink *sink, int irq_num)
  {
    m_callbacks.emplace_back(sink, irq_num);
  }
protected:
private:
  std::string m_name;
  bool m_enabled;
  using irq_callback_t = std::tuple<IInterruptSink*, int>;
  std::vector<irq_callback_t> m_callbacks;
};