#include "rp2040/peri/dma/dma.hpp"

#include <cassert>
#include <tuple>

using namespace RP2040;

DMA::Channel::Channel(DMA &dma, uint32_t id)
: m_dma{dma}
, m_dreq{m_dma.get_dreq(DMA::DReqNum(0))}
, m_enabled{false}
, m_read_addr{0}
, m_read_increment{0}
, m_read_wrap{0}
, m_write_addr{0}
, m_write_increment{0}
, m_write_wrap{0}
, m_transfer_size{TransferSize::BYTE}
, m_transfer_count{0}
, m_transfer_count_reload{0}
, m_reversed{false}
, m_id{id}
, m_err_bits{0}
, m_chain_to{id}
, m_in_flight{false}
, m_running{false}
{
  std::cout << "Channel " << &m_dma.get_dreq(DMA::pio0_rx0) << std::endl;

}

void DMA::DMA::tick(){
  if (!(m_low_priority_mask | m_high_priority_mask)) return;
  m_treq_timer0.tick();
  m_treq_timer1.tick();
  m_treq_timer2.tick();
  m_treq_timer3.tick();
  m_treq_permanent.tick();
  if (m_write_waiting_on_tick){
    m_write_waiting_on_tick = false;
    // std::cout << "DMA::DMA::tick() m_write_master.resume()" << std::endl;
    m_write_master.resume();
  } 
  if (m_read_waiting_on_tick){
    m_read_waiting_on_tick = false;
    // std::cout << "DMA::DMA::tick() m_read_master.resume()" << std::endl;
    m_read_master.resume();
  }
  // update the TREQ dividers first!
  assert((m_low_priority_mask & m_high_priority_mask) == 0 && "DMA::tick() has overlapping high and low priority channels!");
  for (auto mask : {m_high_priority_mask, m_low_priority_mask}){
    for(int i = 0; mask; ++i, mask >>= 1){
      if(mask & 1){
        m_channels[i].tick();
      }
    }
  }
}

void DMA::Channel::tick(){
  if (!enabled()){
    // std::cout << "Channel::tick() should not be called if channel not enabled!" << std::endl;
    // assert(false && "Channel::tick() should not be called if channel not enabled!");
    return;
  }
  if (!busy() || in_flight()){
    return;
  }
  // std::cout << "Channel::tick() " << m_id  << " dreq count " << m_dreq.operator unsigned int() << std::endl;
  if (m_dreq && try_schedule()) {
    m_dreq--;
  } else {
    return;
  }

}

bool DMA::Channel::try_schedule(){
  // std::cout << "Channel::try_schedule() " << m_transfer_count << std::endl;
  if (!m_dma.schedule(m_read_addr, m_write_addr, m_transfer_size, m_reversed, m_id))
    return false; // try again later
  // std::cout << "Ok" << std::endl;
  m_in_flight++;
  return true;
}

void DMA::Channel::complete(){
  if (m_chain_to != m_id) {
    m_dma.chain_trigger(m_chain_to);
  }
  m_running = false;
  // if (!irq_quiet()) {
  //   m_irq.raise();
  // }
}

bool DMA::DMA::schedule(uint32_t read_addr, uint32_t write_addr, TransferSize transfer_size, bool reversed, uint32_t channel_id){
  if (m_addr_fifo.full()){
    // std::cout << "m_addr_fifo full" << std::endl;
    return false;
  }
  m_addr_fifo.push(std::make_tuple(read_addr, write_addr, transfer_size, channel_id, reversed));
  return true;
}

inline static uint32_t reverse_bytes(uint32_t data){
  return ((data & 0xFF) << 24) | ((data & 0xFF00) << 8) | ((data & 0xFF0000) >> 8) | ((data & 0xFF000000) >> 24);
}

BusMaster DMA::DMA::read_master(){
  // std::cout << "read_master" << std::endl;
  while(true){
    do
      co_await next_read_tick();
    while (m_addr_fifo.empty());
    // std::cout << "DMA::DMA::read_master()" << std::endl;
    auto [read_addr, write_addr, transfer_size, channel_id, reversed] = m_addr_fifo.pop();
    uint32_t transfer_data;
    try{
      switch(transfer_size) {
      case TransferSize::BYTE: {
        uint8_t data;
        data = co_await m_ahb.read_byte(read_addr);
        transfer_data = data | data << 8 | data << 16 | data << 24;
      } break;
      case TransferSize::HALFWORD: {
        uint16_t data;
        data = co_await m_ahb.read_halfword(read_addr);
        transfer_data = data | data << 16;
      } break;
      case TransferSize::WORD: {
        uint32_t data;
        data = co_await m_ahb.read_word(read_addr);
        transfer_data = data;
      } break;
      }
    } catch(...){
      m_channels[channel_id].read_error_occurred();
      continue;
    }
    if (reversed)
      transfer_data = reverse_bytes(transfer_data);
    if (sniff_enabled() && sniff_channel() == channel_id && m_channels[channel_id].sniff_enabled()) {
      sniff_consume(transfer_data);
    }
    while (m_data_fifo.full())
      co_await next_read_tick();
    m_data_fifo.push(std::make_tuple(write_addr, transfer_data, transfer_size, channel_id));
  }
}

BusMaster DMA::DMA::write_master(){
  while(true){
    do
      co_await next_write_tick();
    while(m_data_fifo.empty());
    auto [write_addr, data, transfer_size, channel_id] = m_data_fifo.pop();
    // std::cout << "DMA::DMA::write_master(" << std::hex << write_addr << "," << data << ")" << std::endl;
    try{
      switch(transfer_size){
        case TransferSize::BYTE:{
          co_await m_ahb.write_byte(write_addr, data);
        } break;
        case TransferSize::HALFWORD:{
          co_await m_ahb.write_halfword(write_addr, data);
        } break;
        case TransferSize::WORD:{
          co_await m_ahb.write_word(write_addr, data);
        } break;
      }
    } catch(...){
      m_channels[channel_id].write_error_occurred();
      continue;
    }
    m_channels[channel_id].word_transfer_complete();
  }
}

#define ENUM_DMA_CHANNELS(o) \
  o(0)                       \
  o(1)                       \
  o(2)                       \
  o(3)                       \
  o(4)                       \
  o(5)                       \
  o(6)                       \
  o(7)                       \
  o(8)                       \
  o(9)                       \
  o(10)                      \
  o(11) 

#define GROUP_REGS(o, n, m, a, b, c, d) \
  o(n, m, 0, a)                         \
  o(n, m, 1, b)                         \
  o(n, m, 2, c)                         \
  o(n, m, 3, d)

#define ENUM_ALT_REGS(o, n)                                             \
  GROUP_REGS(o, n, 0, READ_ADDR, WRITE_ADDR , TRANS_COUNT, CTRL       ) \
  GROUP_REGS(o, n, 1, CTRL     , READ_ADDR  , WRITE_ADDR , TRANS_COUNT) \
  GROUP_REGS(o, n, 2, CTRL     , TRANS_COUNT, READ_ADDR  , WRITE_ADDR ) \
  GROUP_REGS(o, n, 3, CTRL     , WRITE_ADDR , TRANS_COUNT, READ_ADDR  )

#define ENUM_ALT_REGS_READ(n)  ENUM_ALT_REGS(EVAL_DMA_CHANNELS_READ, n)
#define ENUM_ALT_REGS_WRITE(n) ENUM_ALT_REGS(EVAL_DMA_CHANNELS_WRITE, n)
 

#define EVAL_DMA_CHANNELS_READ(chan, alt, regno, reg) \
  case ((regno*0x4) + (alt*0x10) + (chan*0x40)): {out = dma_reg_read(chan, reg); return PortState::SUCCESS;}
#define EVAL_DMA_CHANNELS_WRITE(chan, alt, regno, reg) \
  case ((regno*0x4) + (alt*0x10) + (chan*0x40)): {dma_reg_write(chan, reg, in, (regno==3)); return PortState::SUCCESS;}



PortState DMA::DMA::read_word_internal(uint32_t addr, uint32_t &out){
  // std::cout << "DMA::DMA::read_word_internal(" << std::hex << addr << ")" << std::endl;
  switch(addr & 0x0000'0fff) {
    ENUM_DMA_CHANNELS(ENUM_ALT_REGS_READ)
    case 0x400: /*INTR*/
    {
      out = m_intr;
    } break;
    default: assert(false);
  }
  // throw std::runtime_error("DMA::DMA::read_word_internal not implemented");
  return PortState::SUCCESS;
}

PortState DMA::DMA::write_word_internal(uint32_t addr, uint32_t in){
  // std::cout << "DMA::DMA::write_word_internal(" << std::hex << addr << ", " << in << ")" << std::endl;
  std::cout << (addr&0x0000'0fff) << std::endl;
  uint32_t v = addr&0x000'0fff;
  switch(v) {
    ENUM_DMA_CHANNELS(ENUM_ALT_REGS_WRITE)
    case 0x40c: /*INTS0*/
    {
      m_intr &= ~in;
    } break;
    case 0x430: /*MULTI_CHAN_TRIGGER*/
    {
      for (int i = 0; i < 12; i++) {
        if (in & (1 << i)) {
          m_channels[i].trigger(true);
        }
      }
    } break;
    default: 
      std::cout << "DMA::DMA::write_word_internal(" << std::hex << addr << ", " << in << ")" << std::endl;
      assert(false);
  }
  // throw std::runtime_error("DMA::DMA::write_word_internal not implemented");
  return PortState::SUCCESS;
}

uint32_t DMA::DMA::read_word_internal_pure(uint32_t addr) const{
  throw std::runtime_error("DMA::DMA::read_word_internal_pure not implemented");
}

inline uint32_t DMA::DMA::dma_reg_read(uint32_t channel, DMARegister reg){
  std::cout << "DMA::DMA::dma_reg_read(" << channel << ", " << reg << ")" << std::endl;
  switch(reg){
    case READ_ADDR: return m_channels[channel].read_addr();
    case WRITE_ADDR: return m_channels[channel].write_addr();
    case TRANS_COUNT: return m_channels[channel].transfer_count();
    case CTRL: return m_channels[channel].control();
    default: assert(false);
  }
}

inline void DMA::DMA::dma_reg_write(uint32_t channel, DMARegister reg, uint32_t in, bool trigger){
  std::cout << "DMA::DMA::dma_reg_write(" << channel << ", " << reg << ", " << in << ", " << trigger << ")" << std::endl;
  switch(reg){
    case READ_ADDR: m_channels[channel].read_addr(in); break;
    case WRITE_ADDR: m_channels[channel].write_addr(in); break;
    case TRANS_COUNT: m_channels[channel].transfer_count(in); break;
    case CTRL: m_channels[channel].control(in); break;
    default: assert(false);
  }
  if (trigger)
    m_channels[channel].trigger(in != 0);
}


void DMA::Channel::control(uint32_t data) {
  m_reg_ctrl = data & 0x00ff'ffff;
  m_err_bits &= ~(data&0xe000'0000);
  m_enabled = data & 1;
  m_dma.set_priority(m_id, high_priority());
  m_transfer_size = static_cast<TransferSize>((data >> 2) & 3);
  m_read_increment = (data & 0x10) ? (1<<m_transfer_size) : 0;
  m_write_increment = (data & 0x20) ? (1<<m_transfer_size) : 0;
  m_read_wrap = 0xffff'ffff;
  m_write_wrap = 0xffff'ffff;
  if (data & 0x2c0) {
    uint32_t wrap = (1 << ((data >> 6) & 0xf))-1;
    if (data & 0x400) m_write_wrap = wrap;
    else m_read_wrap = wrap;
  }
  m_chain_to = (data >> 11) & 0xf;
  uint32_t treq_sel = (data >> 15) & 0x3f;
  set_treq(treq_sel);
  m_irq_quiet = data & 0x0020'0000;
  m_reversed = data & 0x0040'0000;
  m_sniff_enabled = data & 0x0080'0000;
}

void DMA::Channel::set_treq(uint32_t treq){
  DReqSource &source = m_dma.get_dreq(::RP2040::DMA::DMA::DReqNum(treq));
  std::cout << "SET TREQ " << &source << " " << this << std::endl;
  m_dreq.connect(source);
  m_dreq.sync();
  std::cout << "SYNC " << m_id 
            << " " << m_dreq.operator unsigned int() 
            << " " << &m_dreq.source()
            << " " << this
            << std::endl;
}

void DMA::Channel::irq(){
  m_dma.irq(m_id);
}