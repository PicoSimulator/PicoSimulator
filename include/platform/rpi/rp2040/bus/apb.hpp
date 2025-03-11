#pragma once

#include "memory_device.hpp"
#include "platform/rpi/rp2040/peri/resets.hpp"
#include "platform/rpi/rp2040/peri/psm.hpp"
#include "platform/rpi/rp2040/peri/vreg.hpp"
#include "platform/rpi/rp2040/peri/clocks.hpp"
#include "platform/rpi/rp2040/peri/syscfg.hpp"
#include "platform/rpi/rp2040/peri/io_bank0.hpp"
#include "platform/rpi/rp2040/peri/pads_bank0.hpp"
#include "platform/rpi/rp2040/peri/io_qspi.hpp"
#include "platform/rpi/rp2040/peri/pads_qspi.hpp"
#include "platform/rpi/rp2040/peri/watchdog.hpp"
#include "platform/rpi/rp2040/peri/tbman.hpp"
#include "platform/rpi/rp2040/peri/xosc.hpp"
#include "platform/rpi/rp2040/peri/timer.hpp"
#include "platform/rpi/rp2040/peri/pll.hpp"
#include "platform/rpi/rp2040/peri/uart.hpp"
#include "platform/rpi/rp2040/peri/spi.hpp"
#include "platform/rpi/rp2040/peri/i2c.hpp"
#include "platform/rpi/rp2040/peri/rtc.hpp"
#include "platform/rpi/rp2040/peri/dma/dreq.hpp"
#include "platform/rpi/rp2040/peri/rosc.hpp"
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
      SysCfg &syscfg,
      std::array<std::reference_wrapper<GPIO>, 30> bank0_gpios,
      std::array<std::reference_wrapper<GPIO>, 6> qspi_gpios,
      std::array<std::reference_wrapper<InterruptSource>, 4> timer_irqs,
      InterruptSource &uart0_irq,
      InterruptSource &uart1_irq,
      InterruptSource &spi0_irq,
      InterruptSource &spi1_irq,
      InterruptSource &i2c0_irq,
      InterruptSource &i2c1_irq,
      InterruptSource &rtc_irq
    ) 
    : m_runner{bus_task().get_handle()}
    , m_resets{resets}
    , m_vreg{vreg}
    , m_clocks{clocks}
    , m_syscfg{syscfg}
    , m_io_b0{bank0_gpios}
    , m_io_qspi{qspi_gpios}
    , m_uart0{"UART0", uart0_irq}
    , m_uart1{"UART1", uart1_irq}
    , m_spi0{spi0_irq}
    , m_spi1{spi1_irq}
    , m_timer{timer_irqs}
    , m_i2c0{i2c0_irq}
    , m_i2c1{i2c1_irq}
    , m_rtc{rtc_irq}
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
    SPI &spi0() { return m_spi0; }
    SPI &spi1() { return m_spi1; }
    I2C &i2c0() { return m_i2c0; }
    I2C &i2c1() { return m_i2c1; }
    IOQSPI &io_qspi() { return m_io_qspi; }
    RTC &rtc() { return m_rtc; }
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
        // std::cout << "APB completing" << std::endl;
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
    PoweronStateMachine m_psm;
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
    SPI m_spi0, m_spi1;
    I2C m_i2c0, m_i2c1;
    RTC m_rtc;
    ::RP2040::Peripheral::ROsc m_rosc;
  };
}