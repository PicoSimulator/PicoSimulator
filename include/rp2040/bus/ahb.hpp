#pragma once

#include "memory_device.hpp"
#include "clock.hpp"
#include "async.hpp"
#include "tracing/vcd.hpp"

#include <tuple>
#include <array>
#include <queue>
#include <coroutine>

namespace RP2040{
  class RP2040;
}

#define REPEAT_10(...) __VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__,__VA_ARGS__

namespace RP2040::Bus{

    class AHB final: public IAsyncReadWritePort<uint32_t>, public IClockable{
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
      struct MemOpCompare{
        bool operator()(const std::reference_wrapper<MemoryOperation> &lhs, const std::reference_wrapper<MemoryOperation> &rhs) const {
          return m_ahb.bus_priority(lhs.get().upstream_id) < m_ahb.bus_priority(rhs.get().upstream_id);
        }
        AHB &m_ahb;
      };
      using Queue = std::priority_queue<
        std::reference_wrapper<MemoryOperation>,
        std::deque<std::reference_wrapper<MemoryOperation>>,
        MemOpCompare
      >;
      using MemOpItem = std::pair<
        Queue,
        bool
      >;
      using MemOpArray = std::array<
        MemOpItem,
        int(BusDevice::DEVICE_MAX)
      >;
    public:
      AHB(RP2040 &rp2040)
      : m_busOps{
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false},
        MemOpItem{Queue{MemOpCompare{*this}}, false}
      }
      , m_rp2040{rp2040}
      , m_runners{bus_task(0).get_handle(), bus_task(1).get_handle(), bus_task(2).get_handle(), bus_task(3).get_handle()}
      {
        m_vcd.add_item(m_port_address[0]);
        m_vcd.add_item(m_port_address[1]);
        m_vcd.add_item(m_port_address[2]);
        m_vcd.add_item(m_port_address[3]);
      }
      virtual void tick() override;
      Tracing::VCD::Module &vcd() { return m_vcd; }
    protected:
      virtual bool register_op(MemoryOperation &op) override final;
      // virtual void deregister_op(MemoryOperation &op) override final;
      Awaitable<uint8_t> read_byte_internal(uint32_t addr);
      Awaitable<uint16_t> read_halfword_internal(uint32_t addr);
      Awaitable<uint32_t> read_word_internal(uint32_t addr);
      Awaitable<void> write_byte_internal(uint32_t addr, uint8_t in);
      Awaitable<void> write_halfword_internal(uint32_t addr, uint16_t in);
      Awaitable<void> write_word_internal(uint32_t addr, uint32_t in);
      uint32_t bus_priority(uint32_t upstream_id) const
      {
        return (m_BUS_PRIORITY >> upstream_id*4)&1;
      }
    private:
      bool m_waiting_on_ops[4] = {false,false,false,false};
      auto next_op(uint32_t id)
      {
        // std::cout << "AHB::next_op()" << std::endl;
        struct awaitable{
          awaitable(AHB &ahb, uint32_t id)
           : m_ahb{ahb}
           , m_id{id}
           {}
          bool await_ready() { 
            return false;
            return m_ahb.m_ops[m_id] != nullptr;
          }
          void await_suspend(std::coroutine_handle<> h) {
            m_ahb.m_waiting_on_ops[m_id] = true;
            if (m_ahb.m_ops[m_id] != nullptr) {
              auto *op = m_ahb.m_ops[m_id];
              m_ahb.m_ops[m_id] = nullptr;
              
              auto [dev, addr] = m_ahb.lookupBusDeviceAddress(op->addr);
              auto &[q, busy] = m_ahb.m_busOps[dev];
              busy = false;
              // std::cout << "Mem Op Complete AHB: " << std::dec << dev << std::endl;
              op->complete();
            }
          }
          MemoryOperation& await_resume() { 
            m_ahb.m_waiting_on_ops[m_id] = false;
            return *m_ahb.m_ops[m_id]; 
          }
          AHB &m_ahb;
          uint32_t m_id;
        };
        return awaitable{*this, id};
      }

      std::tuple<BusDevice, uint32_t> lookupBusDeviceAddress(uint32_t addr) const;
      MemOpArray m_busOps;
      RP2040 &m_rp2040;

      Task bus_task(uint32_t id) ;
      std::coroutine_handle<> m_runners[4];
      MemoryOperation *m_ops[4];

      uint32_t m_BUS_PRIORITY;

      Tracing::VCD::Register<uint32_t> m_port_address[4] = {
        {"CORE0_ADDR", 32},
        {"CORE1_ADDR", 32},
        {"DMA_R_ADDR", 32},
        {"DMA_W_ADDR", 32},
      };
      Tracing::VCD::Module m_vcd{"AHB"};

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