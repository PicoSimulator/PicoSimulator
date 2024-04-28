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
#include "rp2040/core/fifo.hpp"
#include "rp2040/core/divider.hpp"
#include "rp2040/peri/resets.hpp"
#include "rp2040/peri/vreg.hpp"
#include "rp2040/peri/clocks.hpp"
#include "rp2040/peri/xip.hpp"
#include "rp2040/peri/ssi.hpp"
#include <coroutine>

namespace RP2040{

  class RP2040 final: public IResettable{
  public:
    RP2040();
    virtual void reset() override;
    void run();
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
      // read fifo
      // write fifo
      // spinlocks
      // GPIO
    };
    ClockDiv clk_ref, clk_sys, clk_peri, clk_rtc, clk_usb, clk_adc, clkc_gpout[4];
    const std::array<uint8_t, 16384> m_ROM;
    std::array<uint8_t, 16384> m_XIP_sram;
    std::array<uint8_t, 65536> m_SRAM0;
    std::array<uint8_t, 65536> m_SRAM1;
    std::array<uint8_t, 65536> m_SRAM2;
    std::array<uint8_t, 65536> m_SRAM3;
    std::array<uint8_t, 4096> m_SRAM4;
    std::array<uint8_t, 4096> m_SRAM5;

    class AHB final: public IAsyncReadWritePort<uint32_t>, public IClockable{
    public:
      AHB(RP2040 &rp2040) : m_rp2040{rp2040}{}
      virtual Awaitable<uint8_t> read_byte(uint32_t addr) override;
      virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override;
      virtual Awaitable<uint32_t> read_word(uint32_t addr) override;
      virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override;
      virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override;
      virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override;
      virtual void tick() override;
    protected:
    private:
      enum BusDevice {
        ROM,
        XIP,
        // SSI, is on APB
        SRAM0,
        SRAM1,
        SRAM2,
        SRAM3,
        SRAM4,
        SRAM5,
        APB,
        AHBLITE,
        DEVICE_MAX,
      };
      std::tuple<BusDevice, uint32_t> lookupBusDeviceAddress(uint32_t addr) const;
      struct BusOp{
        BusOp(BusDevice dev, AHB &ahb) : m_dev{dev}, m_ahb{ahb}{}
        bool await_ready() { return false; }
        void await_suspend(std::coroutine_handle<> h) {
          std::get<0>(m_ahb.m_busOps[m_dev]).push(h);
          // std::cout << "waiting on AHB" << std::endl;
        }
        void await_resume() {}
        BusDevice m_dev;
        AHB &m_ahb;
      };
      BusOp registerBusOp(BusDevice dev) {
        return BusOp{dev, *this};
      }
      std::array<std::tuple<std::queue<std::coroutine_handle<>>>, int(BusDevice::DEVICE_MAX)> m_busOps;
      RP2040 &m_rp2040;

      // ROM (0x0000'0000, 0x0000'4000)
      // XIP (0x1000'0000, 0x1000'0000)
      //   XIP (0x1000'0000, 0x0800'0000)
      //   SSI (0x1800'0000, 0x0800'0000)
      // SRAM (0x2000'0000, 0x1000'0000)
      //   SRAM_STRIPED (0x2000'0000, 0x0004'0000)
      //   SRAM4 (0x2004'0000, 0x0000'1000)
      //   SRAM5 (0x2004'1000, 0x0000'1000)
      //   SRAM0 (0x2100'0000, 0x0001'0000)
      //   SRAM1 (0x2100'1000, 0x0001'0000)
      //   SRAM2 (0x2100'2000, 0x0001'0000)
      //   SRAM3 (0x2100'3000, 0x0001'0000)
      // APB Peripherals (0x4000'0000, 0x1000'0000)
      // AHB-Lite Peripherals (0x5000'0000, 0x1000'0000)
      // IOPORT Registers (0xD000'0000, 0x1000'0000)
      // System Registers (0xE000'0000, 0x2000'0000)

      // only the AHB has to implement bus priority, and even then it's only low or high priority for each master.
      // the only other place priority is important is the SIO where proc0 always has priority over proc1.
    };

    class CoreBus final : public IAsyncReadWritePort<uint32_t>{
    public:
      CoreBus(IOPort &ioport, AHB &ahb) : m_ioport{ioport}, m_ahb{ahb} {}
      virtual Awaitable<uint8_t> read_byte(uint32_t addr) override;
      virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override;
      virtual Awaitable<uint32_t> read_word(uint32_t addr) override;
      virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override;
      virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override;
      virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override;
    protected:
    private:
      IOPort &m_ioport;
      AHB &m_ahb;
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
    XIP m_XIP;
    SSI m_SSI;

    // IOPORT Peripherals
    Core::FiFo m_fifo_01, m_fifo_10;
    IOPort m_ioports[2];
    BusMaster *m_bus_masters[2];
    AHB m_ahb;
    APB m_apb;
    CoreBus m_core_bus[2];
    ARMv6M::ARMv6MCore m_cores[2];
  };

  class RP2040XIP final: public IMemoryDevice{
  public:
    RP2040XIP(RP2040 &rp2040);
    virtual PortState read_byte(uint32_t addr, uint8_t &out) override;
    virtual PortState read_halfword(uint32_t addr, uint16_t &out) override;
    virtual PortState read_word(uint32_t addr, uint32_t &out) override;
    virtual PortState write_byte(uint32_t addr, uint8_t in) override;
    virtual PortState write_halfword(uint32_t addr, uint16_t in) override;
    virtual PortState write_word(uint32_t addr, uint32_t in) override;
  protected:
  private:
    RP2040 &m_rp2040;
  };
}