#pragma once

#include <array>
#include <cstdint>

#include "memory_device.hpp"
#include "util/saturating_counter.hpp"
// #include "util/array_converter.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "rp2040/peri/ssi.hpp"

namespace RP2040{
  class XIP final : public IAsyncReadWritePort<uint32_t>, public IClockable{
  public:
    XIP(SSI &ssi) : m_ssi{ssi} {}
    virtual void tick() override;
    virtual Awaitable<uint8_t> read_byte(uint32_t addr) override;
    virtual Awaitable<uint16_t> read_halfword(uint32_t addr) override;
    virtual Awaitable<uint32_t> read_word(uint32_t addr) override;
    virtual Awaitable<void> write_byte(uint32_t addr, uint8_t in) override;
    virtual Awaitable<void> write_halfword(uint32_t addr, uint16_t in) override;
    virtual Awaitable<void> write_word(uint32_t addr, uint32_t in) override;
  protected:
  private:
    //hack for now to make this work!
    std::array<uint8_t, 0x0100'0000> m_flash;


    std::array<uint8_t, 16384> m_data;
    using cache_tag = std::tuple<bool, uint32_t>;
    using tag_set = std::array<cache_tag, 2>;
    std::array<tag_set, 1024> m_cache_tags;
    saturating_counter<uint32_t> m_hit_counter;
    saturating_counter<uint32_t> m_acc_counter;

    uint32_t m_stream_ctr;
    uint32_t m_stream_addr;
    FiFo<uint32_t, 2> m_stream_fifo;

    SSI &m_ssi;
  };

}