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

namespace RP2040{

  class APB final: public IAsyncReadWritePort<uint32_t>, public IClockable{
  public:
    APB(
      Resets &resets,
      VReg &vreg,
      Clocks &clocks,
      SysCfg &syscfg
    ) 
    : m_resets{resets}
    , m_vreg{vreg}
    , m_clocks{clocks}
    , m_syscfg{syscfg}
    {}
    virtual void tick() override;
    virtual Awaitable<uint8_t> read_byte(uint32_t addr) override;
    virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override;
    virtual Awaitable<uint32_t> read_word(uint32_t addr) override;
    virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override;
    virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override;
    virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override;

    Timer &timer() { return m_timer; }
    UART &uart0() { return m_uart0; }
    UART &uart1() { return m_uart1; }
  protected:
  private:
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