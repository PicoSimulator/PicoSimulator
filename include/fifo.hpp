#pragma once

#include <cstdint>
#include <cassert>
#include "rp2040/peri/dma/dreq.hpp"

template<class T>
class IFiFo {
public:
  virtual void push(T t) = 0;
  virtual T pop() = 0;
  virtual T &peek() = 0;
  virtual const T &peek() const = 0;
  virtual size_t count() const = 0;
  virtual bool empty() const = 0;
  virtual bool full() const = 0;
  virtual size_t size() const = 0;
protected:
private:
};

template<class T, size_t N>
class FiFoBase : public IFiFo<T>{
public:
  FiFoBase() : m_head{0}, m_count{0} {}
  virtual void push(T t) override { return push_internal(t); }
  virtual T pop() override { return pop_internal(); }
  virtual T &peek() override { 
    assert(!empty());
    return m_data[m_head]; 
  }
  virtual const T &peek() const override { 
    assert(!empty());
    return m_data[m_head]; 
  }
  virtual size_t count() const override { return m_count; }
  virtual bool empty() const override { return m_count == 0; }
  virtual bool full() const override { return enabled()?(m_count == N):m_count; }
  virtual size_t size() const override { return N; }

  /**
   * When enabled, the FiFo behaves normally.
   * When disabled, the FiFo behaves as a 1 element FiFo.
  */
  void enable(bool en = true) { m_enabled = en; }
  void disable() { enable(false); }
  bool enabled() const { return m_enabled; }
protected:
  void push_internal(T t) {
    assert (!full());
    if (enabled()){
      m_data[(m_head + m_count++) % N] = t;
    } else {
      m_data[m_head] = t;
      m_count = 1;
    }
  }
  T pop_internal() {
    assert (!empty());
    if (enabled()){
      T t = m_data[m_head];
      m_head = (m_head + 1) % N;
      m_count--;
      return t;
    } else {
      T t = m_data[m_head];
      m_count = 0;
      return t;
    }
  }
private:
  std::array<T, N> m_data;
  size_t m_head;
  size_t m_count;
  bool m_enabled = true;
};

template<class T, size_t N>
class FiFo final : public FiFoBase<T, N> {
};

template<class T, size_t N>
class DReqTxFiFo final : public FiFoBase<T, N>, private RP2040::DMA::DReqSource{
public:
  DReqSource &dreq() { return *this; }
  virtual T pop() override {
    auto t = FiFoBase<T, N>::pop_internal();
    dreq()++;
    return t;
  }
protected:
private:
  void sync() override final {
    dreq() += N-count();
  }
};
template<class T, size_t N>
class DReqRxFiFo final : public FiFoBase<T, N>, private RP2040::DMA::DReqSource{
public:
  DReqSource &dreq() { return *this; }
  virtual void push(T t) override {
    FiFoBase<T, N>::push_internal(t);
    dreq()++;
  }
protected:
private:
  void sync() override final {
    dreq() += N-count();
  }
};