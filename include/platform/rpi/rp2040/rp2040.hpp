#pragma once

namespace RP2040 {
  class RP2040;

}

#include "arch/arm/armv6m/core.hpp"
#include "clock.hpp"
#include "memory_device.hpp"
#include "reset.hpp"
#include "rosc.hpp"
#include <array>
#include <queue>
#include <tuple>
#include "bus.hpp"
#include "tracing/vcd.hpp"
#include "platform/rpi/rp2040/pad.hpp"
#include "platform/rpi/rp2040/gpio.hpp"
#include "platform/rpi/rp2040/bus/apb.hpp"
#include "platform/rpi/rp2040/bus/ahb.hpp"
#include "platform/rpi/rp2040/bus/ahb_lite.hpp"
#include "platform/rpi/rp2040/core/fifo.hpp"
#include "platform/rpi/rp2040/core/divider.hpp"
#include "platform/rpi/rp2040/core/spinlock.hpp"
#include "platform/rpi/rp2040/peri/resets.hpp"
#include "platform/rpi/rp2040/peri/vreg.hpp"
#include "platform/rpi/rp2040/peri/clocks.hpp"
#include "platform/rpi/rp2040/peri/xip.hpp"
#include "platform/rpi/rp2040/peri/ssi.hpp"
#include "platform/rpi/rp2040/peri/dma/dma.hpp"
#include "platform/rpi/rp2040/peri/dma/dreq.hpp"
#include <coroutine>

namespace RP2040{

  class RP2040 final: public IResettable, public IClockable, public IODevice{
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
    void tick() override;
    ARMv6M::ARMv6MCore &core0() { return m_cores[0]; }
    ARMv6M::ARMv6MCore &core1() { return m_cores[1]; }
    UART &UART0() {return m_apb.uart0(); }
    UART &UART1() {return m_apb.uart1(); }
    SPI &SPI0() {return m_apb.spi0(); }
    SPI &SPI1() {return m_apb.spi1(); }
    Bus::APB &APB() { return m_apb; }
    ::RP2040::XIP &XIP() { return m_XIP; }
    ::RP2040::SSI &SSI() { return m_SSI; }
    Bus::AHBLite &AHBLite() { return m_ahb_lite; }
    ClockDiv &clk_gpout0() { return clkc_gpout[0]; }
    ClockDiv &clk_gpout1() { return clkc_gpout[1]; }
    ClockDiv &clk_gpout2() { return clkc_gpout[2]; }
    ClockDiv &clk_gpout3() { return clkc_gpout[3]; }

    Tracing::VCD::Module &vcd() { return m_vcd; }
    

    auto &ROM() { return m_ROM; }
    auto &SRAM0() { return m_SRAM0; }
    auto &SRAM1() { return m_SRAM1; }
    auto &SRAM2() { return m_SRAM2; }
    auto &SRAM3() { return m_SRAM3; }
    auto &SRAM4() { return m_SRAM4; }
    auto &SRAM5() { return m_SRAM5; }

    uint64_t tickcnt() const { return m_tickcnt; }


    bool set_param(const std::string &name, const std::string &value) override{
      if(name == "flash"){
        auto &_flash = Simulation::get().components().at(value);
        auto *flash = dynamic_cast<W25QFlash *>(_flash.get());
        m_SSI.set_spidev(flash);
        return true;
      }
      if (name == "uart0"){
        UART0().open(value);
        return true;
      }
      if (name == "uart1"){
        UART1().open(value);
        return true;
      }
      if (name == "seed"){
        // seed the ROSC 
        unsigned seed = std::atoi(value.c_str());
        if (seed == 0)
          return false;
        std::srand(seed);
        m_rosc.set_freq(uint32_t(6'500'000 + (std::rand()-(RAND_MAX/2))/500));
        return true;
      }
      return false;
    }
    void ready() override
    {
      m_rosc.start();
      reset();
    }
  protected:
  private:
    class IOPort final: public IReadWritePort<uint32_t>{
    public:
      IOPort(uint32_t cpuid, Core::FiFo &tx_fifo, Core::FiFo &rx_fifo, std::array<std::reference_wrapper<GPIOSignal>, 30> sio, std::array<std::reference_wrapper<GPIOSignal>, 6> sio_hi)
      : m_cpuid{cpuid}
      , m_tx_fifo{tx_fifo}
      , m_rx_fifo{rx_fifo}
      , m_divider{}
      , m_spinlocks{}
      , m_sio{sio}
      , m_sio_hi{sio_hi}
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
      std::array<std::reference_wrapper<GPIOSignal>, 30> m_sio;
      std::array<std::reference_wrapper<GPIOSignal>, 6> m_sio_hi;
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
      CoreBus(IOPort &ioport, Bus::AHB &ahb, uint32_t id)
      : m_ioport{ioport}
      , m_ahb{ahb}
      , m_op{nullptr}
      , m_runner{bus_task().get_handle()}
      , m_core_id{id}
      {}
      Awaitable<uint8_t> read_byte_internal(uint32_t addr);
      Awaitable<uint16_t> read_halfword_internal(uint32_t addr);
      Awaitable<uint32_t> read_word_internal(uint32_t addr);
      Awaitable<void> write_byte_internal(uint32_t addr, uint8_t in);
      Awaitable<void> write_halfword_internal(uint32_t addr, uint16_t in);
      Awaitable<void> write_word_internal(uint32_t addr, uint32_t in);
    protected:
      virtual bool register_op(MemoryOperation &op) override final{
        // std::cout << "CoreBus Registering op" << std::hex << (uintptr_t)this << ":" << uintptr_t(&op) << "(" << m_core_id << ")" << std::endl;
        assert(m_op == nullptr);
        op.upstream_id = m_core_id;
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
      uint32_t m_core_id;
    };

    // keep a list of the Bus Masters
    // these are coroutine tasks
    // they may be blocked on a memory access.
    // once all tasks are suspended memory access can be performed.

    ROsc m_rosc;

    // GPIO Pads
    Pad m_pads_bank0[32];
    Pad m_pads_qspi[6];
    GPIO m_gpio_bank0[30];// SWDIO,SWCLK have pads but no GPIO connections!
    GPIO m_gpio_qspi[6];
    std::array<GPIOSignal, 30> m_sio;
    std::array<GPIOSignal, 6> m_sio_hi;

    // APB Peripherals
    Resets m_resets;
    VReg m_vreg;
    Clocks m_clocks;
    SysCfg m_syscfg;

    // AHB Peripherals
    ::RP2040::XIP m_XIP;
    ::RP2040::SSI m_SSI;

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

    Tracing::VCD::Module m_vcd;
  };
}