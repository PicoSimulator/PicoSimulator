#pragma once

#include "async.hpp"
#include "bus.hpp"
#include "clock.hpp"
#include "fifo.hpp"
#include "platform/rpi/rp2040/peripheral.hpp"
#include "platform/rpi/rp2040/bus/ahb.hpp"
#include "reset.hpp"

#include "dreq.hpp"
#include "treq.hpp"

#include <array>
#include <coroutine>

#define ENUM_DREQS(o) \
o(0, pio0_tx0) \
o(1, pio0_tx1) \
o(2, pio0_tx2) \
o(3, pio0_tx3) \
o(4, pio0_rx0) \
o(5, pio0_rx1) \
o(6, pio0_rx2) \
o(7, pio0_rx3) \
o(8, pio1_tx0) \
o(9, pio1_tx1) \
o(10, pio1_tx2) \
o(11, pio1_tx3) \
o(12, pio1_rx0) \
o(13, pio1_rx1) \
o(14, pio1_rx2) \
o(15, pio1_rx3) \
o(16, spi0_tx) \
o(17, spi0_rx) \
o(18, spi1_tx) \
o(19, spi1_rx) \
o(20, uart0_tx) \
o(21, uart0_rx) \
o(22, uart1_tx) \
o(23, uart1_rx) \
o(24, pwm_wrap0) \
o(25, pwm_wrap1) \
o(26, pwm_wrap2) \
o(27, pwm_wrap3) \
o(28, pwm_wrap4) \
o(29, pwm_wrap5) \
o(30, pwm_wrap6) \
o(31, pwm_wrap7) \
o(32, i2c0_tx) \
o(33, i2c0_rx) \
o(34, i2c1_tx) \
o(35, i2c1_rx) \
o(36, adc) \
o(37, xip_stream) \
o(38, xip_ssitx) \
o(39, xip_ssirx) \

namespace RP2040::DMA{
  enum TransferSize{
    BYTE = 0,
    HALFWORD = 1,
    WORD = 2,
  };
  class DMA;
  class Channel final : public IClockable{
  public:
    Channel(DMA &dma, uint32_t id);
    virtual void tick() override;
    void abort() { m_running  = false; }
    bool enabled() const { return m_enabled; }
    void read_error_occurred() { m_err_bits |= 0xc000'0000; abort(); irq(); }
    void write_error_occurred() { m_err_bits |= 0xa000'0000; abort(); irq(); }
    void enable_read_wrap(uint32_t log2) { m_read_wrap = (1u << log2) - 1; }
    void enable_write_wrap(uint32_t log2) { m_write_wrap = (1u << log2) - 1; }
    void disable_wrap() { m_read_wrap = m_write_wrap = 0; }
    void set_read_increment(uint32_t increment) { m_read_increment = increment; }
    void set_write_increment(uint32_t increment) { m_write_increment = increment; }
    uint32_t in_flight() const { return m_in_flight; }
    void word_transfer_complete() {
      m_read_addr = (m_read_addr & ~m_read_wrap) + ((m_read_addr + m_read_increment) & m_read_wrap);
      m_write_addr = (m_write_addr & ~m_write_wrap) + ((m_write_addr + m_write_increment) & m_write_wrap);
      m_transfer_count--;
      m_in_flight--;
      if (m_transfer_count == 0)
        complete();
    }
    // bool busy() const { return false;}
    bool busy() const { return m_transfer_count != 0 && m_running;}
    void trigger(bool nnull = true) { 
      if (!nnull) {
        irq();
        return;
      }
      if (busy()) return;
      m_running = true; 
      m_transfer_count = m_transfer_count_reload;
      m_dreq.sync();
    }
    bool high_priority() const { return m_reg_ctrl & 0x0000'0002; }
    bool irq_quiet() const { return m_reg_ctrl & 0x0020'0000; }
    bool bswap() const { return m_reg_ctrl & 0x0040'0000; }
    bool sniff_enabled() const { return m_reg_ctrl & 0x0080'0000; }
    uint32_t read_addr() const { return m_read_addr; }
    void read_addr(uint32_t addr) { m_read_addr = addr; }
    uint32_t write_addr() const { return m_write_addr; }
    void write_addr(uint32_t addr) { m_write_addr = addr; }
    uint32_t transfer_count() const { return m_transfer_count; }
    void transfer_count(uint32_t count) { m_transfer_count_reload = count; }
    uint32_t control() const { 
      return 
        m_err_bits 
        | (busy()?0x0100'0000:0) 
        | m_reg_ctrl;
        /* 
        | (sniff_enabled()?0x0080'0000:0) 
        | (bswap()?0x0040'0000:0) 
        | (irq_quiet()?0x0020'0000:0)
        | treq_sel() << 15
        | chain_to() << 11
        | ring_sel() << 10
        | ring_size() << 6
        | incr_write() << 5
        | incr_read() << 4
        | data_size() << 2
        | high_priority() << 1
        | enabled(); */
      }
    void control(uint32_t data);
    void set_treq(uint32_t treq);
  protected:
  private:
    void irq();
    bool try_schedule();
    void complete();
    void reload() {
      m_transfer_count = m_transfer_count_reload;
      m_in_flight = false;
    }
    DMA &m_dma;
    DReqSink m_dreq;
    bool m_enabled;
    uint32_t m_read_addr, m_read_increment, m_read_wrap;
    uint32_t m_write_addr, m_write_increment, m_write_wrap;
    TransferSize m_transfer_size;
    uint32_t m_transfer_count, m_transfer_count_reload;
    bool m_reversed;
    const uint32_t m_id;
    uint32_t m_err_bits;
    uint32_t m_chain_to;
    uint32_t m_in_flight;
    bool m_running;
    bool m_sniff_enabled;
    bool m_irq_quiet;
    uint32_t m_reg_ctrl;
  };

  enum DMARegister {
    READ_ADDR,
    WRITE_ADDR,
    TRANS_COUNT,
    CTRL,
  };

  class TReqTimer final : public IClockable, public IResettable, public DReqSource {
  public:
    virtual void tick() override { DReqSource::operator++(); }
    virtual void sync() override { }
    virtual void reset() override {};
  private:
    uint16_t m_idiv, m_fdiv;
  };

  class DMA final : public IClockable, public IPeripheralPort{
  public:
    using dreq_list_t = std::array<std::reference_wrapper<DReqSource>, 40>;
    DMA(Bus::AHB &ahb, dreq_list_t dreqs)
    : m_read_master{read_master()}
    , m_write_master{write_master()}
    , m_dreqs{dreqs}
    , m_channels{{{*this, 0}, {*this, 1}, {*this, 2}, {*this, 3}, {*this, 4}, {*this, 5}, {*this, 6}, {*this, 7}, {*this, 8}, {*this, 9}, {*this, 10}, {*this, 11}}}
    , m_ahb{ahb}
    {
      std::cout << "DMA" << std::hex << &dreq_pio0_rx0() << std::endl;
      

    }
  #define DREQ_EVAL(a, b) b = a,
    enum DReqNum{
      ENUM_DREQS(DREQ_EVAL)
      DREQ_MAX,
      dma_timer0 = 0x3b,
      dma_timer1 = 0x3c,
      dma_timer2 = 0x3d,
      dma_timer3 = 0x3e,
      dma_permanent = 0x3f,
    };
  #undef DREQ_EVAL
    DReqSource &get_dreq(DReqNum dreq) { 
      #define DREQ_EVAL(a,b) case b: return m_dreqs[DReqNum::b];
      switch(dreq){
        ENUM_DREQS(DREQ_EVAL)
        case dma_timer0: return treq_timer0();
        case dma_timer1: return treq_timer1();
        case dma_timer2: return treq_timer2();
        case dma_timer3: return treq_timer3();
        case dma_permanent: return treq_permanent();
        default: return NullDReqSource::get();
      }
      #undef DREQ_EVAL
    }
    const DReqSource &get_dreq(DReqNum dreq) const { return m_dreqs[dreq]; }
  #define DREQ_EVAL(a, b) \
    DReqSource &dreq_##b() { return m_dreqs[DReqNum::b]; } \
    const DReqSource &dreq_##b() const { return m_dreqs[DReqNum::b]; }
    ENUM_DREQS(DREQ_EVAL)
  #undef DREQ_EVAL
    DReqSource &treq_timer0() { return m_treq_timer0; }
    DReqSource &treq_timer1() { return m_treq_timer1; }
    DReqSource &treq_timer2() { return m_treq_timer2; }
    DReqSource &treq_timer3() { return m_treq_timer3; }
    DReqSource &treq_permanent() { return m_treq_permanent; }
    virtual void tick() override;
    bool schedule(uint32_t read_addr, uint32_t write_addr, TransferSize transfer_size, bool reversed, uint32_t channel_id);
    void chain_trigger(uint32_t channel_id) { m_channels[channel_id].trigger(); }
    bool sniff_enabled() const { return true; }
    uint32_t sniff_channel() const { return 0; }
    void set_priority(uint32_t channel, bool high_priority) {
      m_high_priority_mask &= ~(1u << channel);
      m_low_priority_mask &= ~(1u << channel);
      m_high_priority_mask |= high_priority << channel;
      m_low_priority_mask |= !high_priority << channel;
    }
    void irq(uint32_t channel) { m_intr |= 1u << channel; }
  protected:
    virtual PortState read_word_internal(uint32_t addr, uint32_t &out) override;
    virtual PortState write_word_internal(uint32_t addr, uint32_t in) override;
    virtual uint32_t read_word_internal_pure(uint32_t addr) const override;
  private:
    uint32_t dma_reg_read(uint32_t channel, DMARegister reg);
    void dma_reg_write(uint32_t channel, DMARegister reg, uint32_t data, bool trigger);
    auto next_read_tick() { /* std::cout << "next_read_tick()" << std::endl; */ m_read_waiting_on_tick = true; return std::suspend_always{};}
    auto next_write_tick() { /* std::cout << "next_write_tick()" << std::endl; */ m_write_waiting_on_tick = true; return std::suspend_always{};}
    void sniff_consume(uint32_t data) {};
    BusMaster read_master();
    BusMaster write_master();
    BusMaster m_read_master, m_write_master;
    bool m_read_waiting_on_tick = true, m_write_waiting_on_tick = true;
    FiFo<std::tuple<uint32_t, uint32_t, TransferSize, uint32_t, bool>, 16> m_addr_fifo;
    FiFo<std::tuple<uint32_t, uint32_t, TransferSize, uint32_t>, 16> m_data_fifo;
    dreq_list_t m_dreqs;
    TReqTimer m_treq_timer0, m_treq_timer1, m_treq_timer2, m_treq_timer3;
    TReqTimer m_treq_permanent;
    std::array<Channel, 12> m_channels;
    uint32_t m_high_priority_mask;
    uint32_t m_low_priority_mask;
    Bus::AHB &m_ahb;
    uint32_t m_intr = 0;
  };
} // namespace RP2040::DMA