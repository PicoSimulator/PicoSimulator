#pragma once

#include <limits>

template<class T>
class saturating_counter {
public:
  T get() const { return m_counter; }
  void set(T t) { m_counter = t; }
  void reset() { set(0); }
  T operator++() { T v = m_counter; increment(); return v; }
  T operator++(int) { increment(); return m_counter; }
protected:
private:
  void increment() { if (m_counter < std::numeric_limits<T>::max()) m_counter++; }
  T m_counter;
};