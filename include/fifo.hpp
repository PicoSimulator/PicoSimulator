#pragma once

#include <cstdint>
#include "rp2040/peri/dma/dreq.hpp"

template<class T, size_t N>
class FiFo {
public:
  FiFo() : m_head{0}, m_count{0} {}
  void push(T t) {
    if (m_count < N)
      m_data[(m_head + m_count++) % N] = t;
  }
  T pop() {
    if (m_count == 0) return T{};
    T t = m_data[m_head];
    m_head = (m_head + 1) % N;
    m_count--;
    return t;
  }
  T &peek() { return m_data[m_head]; }
  const T &peek() const { return m_data[m_head]; }
  size_t count() const { return m_count; }
  bool empty() const { return m_count == 0; }
  bool full() const { return m_count == N; }
protected:
private:
  std::array<T, N> m_data;
  size_t m_head;
  size_t m_count;
};

template<class T, size_t N>
class DReqTxFiFo final : private RP2040::DMA::DReqSource {
public:
  DReqSource &dreq() { return *this; }
  DReqTxFiFo() : m_head{0}, m_count{0} {}
  void push(T t) {
    if (m_count >= N)
      return;
    m_data[(m_head + m_count++) % N] = t;

  }
  T pop() {
    if (m_count == 0) return T{};
    T t = m_data[m_head];
    m_head = (m_head + 1) % N;
    m_count--;
    dreq()++;
    return t;
  }
  T &peek() { return m_data[m_head]; }
  const T &peek() const { return m_data[m_head]; }
  size_t count() const { return m_count; }
  bool empty() const { return m_count == 0; }
  bool full() const { return m_count == N; }
protected:
private:
  void sync() override final {
    dreq() += N-m_count;
  }
  std::array<T, N> m_data;
  size_t m_head;
  size_t m_count;
};
template<class T, size_t N>
class DReqRxFiFo final : private RP2040::DMA::DReqSource {
public:
  DReqSource &dreq() { return *this; }
  DReqRxFiFo() : m_head{0}, m_count{0} {}
  void push(T t) {
    if (m_count >= N)
      return;
    m_data[(m_head + m_count++) % N] = t;
    dreq()++;

  }
  T pop() {
    if (m_count == 0) return T{};
    T t = m_data[m_head];
    m_head = (m_head + 1) % N;
    m_count--;
    return t;
  }
  T &peek() { return m_data[m_head]; }
  const T &peek() const { return m_data[m_head]; }
  size_t count() const { return m_count; }
  bool empty() const { return m_count == 0; }
  bool full() const { return m_count == N; }
protected:
private:
  void sync() override final {
    dreq() += m_count;
  }
  std::array<T, N> m_data;
  size_t m_head;
  size_t m_count;
};