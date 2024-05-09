#pragma once

#include <limits>

template<class T, T max=std::numeric_limits<T>::max(), T min=std::numeric_limits<T>::min(), T increment_value=1>
class saturating_counter {
public:
  T get() const { return m_counter; }
  operator T() const { return m_counter; }
  void set(T t) { m_counter = t; }
  void reset() { set(0); }
  T operator++() { T v = m_counter; increment(); return v; }
  T operator++(int) { increment(); return m_counter; }
  T operator--() { T v = m_counter; decrement(); return v; }
  T operator--(int) { decrement(); return m_counter; }
  T operator=(T t) { m_counter = t; return m_counter; }
protected:
private:
  void decrement() { if ((m_counter - increment_value) >= min && (m_counter > (m_counter - increment_value))) m_counter -= increment_value; }
  void increment() { if ((m_counter + increment_value) <= max && (m_counter < (m_counter + increment_value))) m_counter += increment_value; }
  T m_counter;
};