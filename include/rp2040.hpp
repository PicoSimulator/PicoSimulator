#pragma once

namespace RP2040 {
  class RP2040;

}

#include "armv6m/core.hpp"
#include "clock.hpp"
#include "memory_device.hpp"
#include "reset.hpp"
#include <array>
#include <queue>
#include <tuple>
#include "bus.hpp"
#include "rp2040/pad.hpp"
#include "rp2040/bus/apb.hpp"
#include "rp2040/bus/ahb.hpp"
#include "rp2040/bus/ahb_lite.hpp"
#include "rp2040/core/fifo.hpp"
#include "rp2040/core/divider.hpp"
#include "rp2040/core/spinlock.hpp"
#include "rp2040/peri/resets.hpp"
#include "rp2040/peri/vreg.hpp"
#include "rp2040/peri/clocks.hpp"
#include "rp2040/peri/xip.hpp"
#include "rp2040/peri/ssi.hpp"
#include "rp2040/peri/dma/dma.hpp"
#include "rp2040/peri/dma/dreq.hpp"
#include <coroutine>

namespace RP2040{

  class RP2040 final: public IResettable{
  public:
    enum IRQ{
      TIMER_IRQ_0 = 0,
      TIMER_IRQ_1 = 1,
      TIMER_IRQ_2 = 2,
      TIMER_IRQ_3 = 3,
      PWM_IRQ_WRAP = 4,
      USBCTRL_IRQ = 5,
      XIP_IRQ = 6,
      PIO0_IRQ_0 = 7,
      PIO0_IRQ_1 = 8,
      PIO1_IRQ_0 = 9,
      PIO1_IRQ_1 = 10,
      DMA_IRQ_0 = 11,
      DMA_IRQ_1 = 12,
      IO_IRQ_BANK0 = 13,
      IO_IRQ_QSPI = 14,
      SIO_IRQ_PROC0 = 15,
      SIO_IRQ_PROC1 = 16,
      CLOCKS_IRQ = 17,
      SPI0_IRQ = 18,
      SPI1_IRQ = 19,
      UART0_IRQ = 20,
      UART1_IRQ = 21,
      ADC_IRQ_FIFO = 22,
      I2C0_IRQ = 23,
      I2C1_IRQ = 24,
      RTC_IRQ = 25,
    };
    RP2040();
    virtual void reset() override;
    void run(unsigned int max_ticks);
    void load_binary(const std::string &path);
    UART &UART0();
    Bus::APB &APB() { return m_apb; }
    ::RP2040::XIP &XIP() { return m_XIP; }
    Bus::AHBLite &AHBLite() { return m_ahb_lite; }

    auto &ROM() { return m_ROM; }
    auto &SRAM0() { return m_SRAM0; }
    auto &SRAM1() { return m_SRAM1; }
    auto &SRAM2() { return m_SRAM2; }
    auto &SRAM3() { return m_SRAM3; }
    auto &SRAM4() { return m_SRAM4; }
    auto &SRAM5() { return m_SRAM5; }

    uint64_t tickcnt() const { return m_tickcnt; }

  protected:
  private:
    class IOPort final: public IReadWritePort<uint32_t>{
    public:
      IOPort(uint32_t cpuid, Core::FiFo &tx_fifo, Core::FiFo &rx_fifo)
      : m_cpuid{cpuid}
      , m_tx_fifo{tx_fifo}
      , m_rx_fifo{rx_fifo}
      {}
      virtual PortState read_byte(uint32_t addr, uint8_t &out) override;
      virtual PortState read_halfword(uint32_t addr, uint16_t &out) override;
      virtual PortState read_word(uint32_t addr, uint32_t &out) override;
      virtual PortState write_byte(uint32_t addr, uint8_t in) override;
      virtual PortState write_halfword(uint32_t addr, uint16_t in) override;
      virtual PortState write_word(uint32_t addr, uint32_t in) override;

    protected:
    private:
    uint32_t m_cpuid;
    Core::FiFo &m_tx_fifo, &m_rx_fifo;
    Core::Divider m_divider;
    Core::Spinlocks m_spinlocks;
      // read fifo
      // write fifo
      // spinlocks
      // GPIO
    };
    InterruptSourceSet m_interrupts;
    ClockDiv clk_ref, clk_sys, clk_peri, clk_rtc, clk_usb, clk_adc, clkc_gpout[4];
    const std::array<uint8_t, 16384> m_ROM;
    std::array<uint8_t, 16384> m_XIP_sram;
    std::array<uint8_t, 65536> m_SRAM0;
    std::array<uint8_t, 65536> m_SRAM1;
    std::array<uint8_t, 65536> m_SRAM2;
    std::array<uint8_t, 65536> m_SRAM3;
    std::array<uint8_t, 4096> m_SRAM4;
    std::array<uint8_t, 4096> m_SRAM5;
    DMA::NullDReqSource m_null_dreq;


    class CoreBus final : public IAsyncReadWritePort<uint32_t>{
    public:
      CoreBus(IOPort &ioport, Bus::AHB &ahb)
      : m_ioport{ioport}
      , m_ahb{ahb}
      , m_op{nullptr}
      , m_runner{bus_task().get_handle()}
      {}
      Awaitable<uint8_t> read_byte_internal(uint32_t addr);
      Awaitable<uint16_t> read_halfword_internal(uint32_t addr);
      Awaitable<uint32_t> read_word_internal(uint32_t addr);
      Awaitable<void> write_byte_internal(uint32_t addr, uint8_t in);
      Awaitable<void> write_halfword_internal(uint32_t addr, uint16_t in);
      Awaitable<void> write_word_internal(uint32_t addr, uint32_t in);
    protected:
      virtual bool register_op(MemoryOperation &op) override final{
        // std::cout << "CoreBus Registering op" << std::hex << (uintptr_t)this << ":" << uintptr_t(&op) << std::endl;
        assert(m_op == nullptr);
        m_op = &op;
        if(m_waiting_on_op)
          m_runner.resume();
        return m_waiting_on_op;
      }
      // virtual void deregister_op(MemoryOperation &op) override final{
      //   // std::cout << "CoreBus deRegistering op" << std::hex << (uintptr_t)this << ":" << uintptr_t(&op) << std::endl;
      //   assert(m_op == &op);
      //   m_op = nullptr;
      //   m_waiting_on_op = false;
      // }
    private:
      MemoryOperation *m_op;
      std::coroutine_handle<> m_runner;
      bool m_waiting_on_op;
      auto next_op(){
        struct awaitable{
          bool await_ready() {
            return false;
            return m_bus.m_op != nullptr;
          }
          MemoryOperation &await_resume() {
            m_bus.m_waiting_on_op = false;
            return *m_bus.m_op;
          }
          void await_suspend(std::coroutine_handle<> handle) {
            m_bus.m_waiting_on_op = true;
            if (m_bus.m_op != nullptr) {
              auto *op = m_bus.m_op;
              m_bus.m_op = nullptr;
              op->complete();
            }
          }
          CoreBus &m_bus;
        };
        return awaitable{*this};
      }
      Task bus_task();
    private:
      IOPort &m_ioport;
      Bus::AHB &m_ahb;
    };

    // keep a list of the Bus Masters
    // these are coroutine tasks
    // they may be blocked on a memory access.
    // once all tasks are suspended memory access can be performed.


    // GPIO Pads
    Pad m_pads_bank0[30];
    Pad m_pads_qspi[6];

    // APB Peripherals
    Resets m_resets;
    VReg m_vreg;
    Clocks m_clocks;
    SysCfg m_syscfg;

    // AHB Peripherals
    ::RP2040::XIP m_XIP;
    SSI m_SSI;

    // IOPORT Peripherals
    Core::FiFo m_fifo_01, m_fifo_10;
    IOPort m_ioports[2];
    BusMaster *m_bus_masters[2];
    Bus::AHBLite m_ahb_lite;
    Bus::AHB m_ahb;
    DMA::DMA m_DMA;
    Bus::APB m_apb;
    CoreBus m_core_bus[2];
    ARMv6M::ARMv6MCore m_cores[2];
    uint64_t m_tickcnt;
  };
}