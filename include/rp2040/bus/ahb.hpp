#pragma once

#include "memory_device.hpp"
#include "clock.hpp"
#include "async.hpp"

#include <tuple>
#include <array>
#include <queue>
#include <coroutine>

namespace RP2040{
  class RP2040;
}

namespace RP2040::Bus{

    class AHB final: public IAsyncReadWritePort<uint32_t>, public IClockable{
    public:
      AHB(RP2040 &rp2040)
      : m_rp2040{rp2040}
      , m_runners{bus_task(0).get_handle(), bus_task(1).get_handle(), bus_task(2).get_handle(), bus_task(3).get_handle()}
      {}
      virtual void tick() override;
    protected:
      virtual void register_op(MemoryOperation &op) override final;
      virtual void deregister_op(MemoryOperation &op) override final;
      Awaitable<uint8_t> read_byte_internal(uint32_t addr);
      Awaitable<uint16_t> read_halfword_internal(uint32_t addr);
      Awaitable<uint32_t> read_word_internal(uint32_t addr);
      Awaitable<void> write_byte_internal(uint32_t addr, uint8_t in);
      Awaitable<void> write_halfword_internal(uint32_t addr, uint16_t in);
      Awaitable<void> write_word_internal(uint32_t addr, uint32_t in);
    private:
      auto next_op(uint32_t id)
      {
        // std::cout << "AHB::next_op()" << std::endl;
        struct awaitable{
          awaitable(AHB &ahb, uint32_t id)
           : m_ahb{ahb}
           , m_id{id}
           {}
          bool await_ready() { return false; }
          void await_suspend(std::coroutine_handle<> h) {
            if (m_ahb.m_ops[m_id] != nullptr) {
              m_ahb.m_ops[m_id]->m_caller.resume();
            }
          }
          MemoryOperation& await_resume() { return *m_ahb.m_ops[m_id]; }
          AHB &m_ahb;
          uint32_t m_id;
        };
        return awaitable{*this, id};
      }
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
      std::array<std::tuple<std::queue<std::reference_wrapper<MemoryOperation>>, bool>, int(BusDevice::DEVICE_MAX)> m_busOps;
      RP2040 &m_rp2040;

      Task bus_task(uint32_t id) ;
      std::coroutine_handle<> m_runners[4];
      MemoryOperation *m_ops[4];

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

      // only the AHB has to implement bus priority, and even then it's only low or high priority for each master.
      // the only other place priority is important is the SIO where proc0 always has priority over proc1.
    };
}