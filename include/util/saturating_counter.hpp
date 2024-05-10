#pragma once

#include <limits>

template<class T, T max=std::numeric_limits<T>::max(), T min=std::numeric_limits<T>::min(), T increment_value=1>
class saturating_counter {
public:
  T get() const { return m_counter; }
  operator T() const { return m_counter; }
  void set(T t) { m_counter = t; }
  void reset() { set(0); }
  T operator++() { T v = *this; increment(); return v; }
  T operator++(int) { increment(); return *this; }
  T operator+=(T v) { add(v); return *this; }
  T operator--() { T v = *this; decrement(); return v; }
  T operator--(int) { decrement(); return *this; }
  T operator-=(T v) { sub(v); return *this; }
  T operator=(T t) { m_counter = t; return *this; }
protected:
private:
  void decrement() { if ((m_counter - increment_value) >= min && (m_counter > (m_counter - increment_value))) m_counter -= increment_value; }
  void add(T v) { if ((max - v) < m_counter) m_counter = max; else m_counter += v; }
  void increment() { if ((m_counter + increment_value) <= max && (m_counter < (m_counter + increment_value))) m_counter += increment_value; }
  void sub(T v) { if ((min + v) > m_counter) m_counter = min; else m_counter -= v;}
  T m_counter;
};