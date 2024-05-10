#pragma once

#include "memory_device.hpp"
#include "rp2040/peri/resets.hpp"
#include "rp2040/peri/vreg.hpp"
#include "rp2040/peri/clocks.hpp"
#include "rp2040/peri/syscfg.hpp"
#include "rp2040/peri/io_bank0.hpp"
#include "rp2040/peri/pads_bank0.hpp"
#include "rp2040/peri/io_qspi.hpp"
#include "rp2040/peri/pads_qspi.hpp"
#include "rp2040/peri/watchdog.hpp"
#include "rp2040/peri/tbman.hpp"
#include "rp2040/peri/xosc.hpp"
#include "rp2040/peri/timer.hpp"
#include "rp2040/peri/pll.hpp"
#include "rp2040/peri/uart.hpp"
#include "rp2040/peri/dma/dreq.hpp"
#include "async.hpp"

#include <cassert>
#include <coroutine>

namespace RP2040::Bus{

  class APB final: public IAsyncReadWritePort<uint32_t>, public IClockable{
  public:
    APB(
      Resets &resets,
      VReg &vreg,
      Clocks &clocks,
      SysCfg &syscfg
    ) 
    : m_runner{bus_task().get_handle()}
    , m_resets{resets}
    , m_vreg{vreg}
    , m_clocks{clocks}
    , m_syscfg{syscfg}
    , m_uart0{}
    , m_uart1{}
    {}
    virtual void tick() override;
    Awaitable<uint8_t> read_byte_internal(uint32_t addr);
    Awaitable<uint16_t> read_halfword_internal(uint32_t addr);
    Awaitable<uint32_t> read_word_internal(uint32_t addr);
    Awaitable<void> write_byte_internal(uint32_t addr, uint8_t in);
    Awaitable<void> write_halfword_internal(uint32_t addr, uint16_t in);
    Awaitable<void> write_word_internal(uint32_t addr, uint32_t in);

    Timer &timer() { return m_timer; }
    UART &uart0() { return m_uart0; }
    UART &uart1() { return m_uart1; }
  protected:
    virtual bool register_op(MemoryOperation &op) override final{
      // std::cout << "APB register " << uintptr_t(&op) << std::endl;
      assert(m_op == nullptr);
      m_op = &op;
      if(m_waiting_on_op)
        m_runner.resume();
      return m_waiting_on_op;
    }
  private:
    MemoryOperation *m_op;
    std::coroutine_handle<> m_runner;
    bool m_waiting_on_op;
    auto next_op(){
      struct awaitable{
        bool await_ready() { \
          return m_bus.m_op != nullptr; 
        }
        MemoryOperation &await_resume() { 
          m_bus.m_waiting_on_op = false;
          return *m_bus.m_op; 
        }
        void await_suspend(std::coroutine_handle<> handle) {
          m_bus.m_waiting_on_op = true;
          m_bus.m_op = nullptr;
        }
        APB &m_bus;
      };
      if (m_op != nullptr) {
        auto *op = m_op;
        m_op = nullptr;
        std::cout << "APB completing" << std::endl;
        op->complete();
      }
      return awaitable{*this};
    }
    Task bus_task();
    // UART0
    // UART1
    // SPI0
    // SPI1
    // I2C0
    // I2C1
    // ADC
    // PWM
    // TIM
    // WDT
    // RTC
    Resets &m_resets;
    VReg &m_vreg;
    Clocks &m_clocks;
    SysCfg &m_syscfg;
    IOBank0 m_io_b0;
    PadsBank0 m_pads_b0;
    Watchdog m_watchdog;
    TBMan m_tbman;
    IOQSPI m_io_qspi;
    PadsQSPI m_pads_qspi;
    XOSC m_xosc;
    Timer m_timer;
    PLL m_pll_sys, m_pll_usb;
    UART m_uart0, m_uart1;
  };
}