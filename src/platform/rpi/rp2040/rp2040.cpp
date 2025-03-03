#include "platform/rpi/rp2040/rp2040.hpp"
#include "platform/rpi/rp2040/bootloader.hpp"
#include <array>
#include <async.hpp>
#include "arch/arm/armv6m/exception.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include "simulation.hpp"

#define ENUM_32(o) \
  o( 0) o( 1) o( 2) o( 3) o( 4) \
  o( 5) o( 6) o( 7) o( 8) o( 9) \
  o(10) o(11) o(12) o(13) o(14) \
  o(15) o(16) o(17) o(18) o(19) \
  o(20) o(21) o(22) o(23) o(24) \
  o(25) o(26) o(27) o(28) o(29) \
  o(30) o(31) 

#define ENUM_30(o) \
  o( 0) o( 1) o( 2) o( 3) o( 4) \
  o( 5) o( 6) o( 7) o( 8) o( 9) \
  o(10) o(11) o(12) o(13) o(14) \
  o(15) o(16) o(17) o(18) o(19) \
  o(20) o(21) o(22) o(23) o(24) \
  o(25) o(26) o(27) o(28) o(29) 

#define ENUM_6(o)\
  o( 0) o( 1) o( 2) o( 3) o( 4) \
  o( 5)

#define DUMMY GPIOSignal::dummy()
#define DUMMY_REMAINDER DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY
#define SPI0(name) m_apb.spi0().name()
#define SPI1(name) m_apb.spi1().name()
#define UART0(name) m_apb.uart0().name()
#define UART1(name) m_apb.uart1().name()
#define I2C0(name) m_apb.i2c0().name()
#define I2C1(name) m_apb.i2c1().name()
#define PWM0(name) DUMMY
#define PWM1(name) DUMMY
#define SIO(n) m_sio[n]
#define SIO_HI(n) m_sio_hi[n]
#define PIO0(n) DUMMY
#define PIO1(n) DUMMY
#define SSI(name) m_SSI.name()

#define GPIO_0_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(RX) , UART0(TX) , I2C0(SDA), PWM0(A), SIO( 0), PIO0( 0), PIO1( 0), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_1_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(CSn), UART0(RX) , I2C0(SCL), PWM0(B), SIO( 1), PIO0( 1), PIO1( 1), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_2_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(SCK), UART0(CTS), I2C1(SDA), PWM1(A), SIO( 2), PIO0( 2), PIO1( 2), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_3_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(TX) , UART0(RTS), I2C1(SCL), PWM1(B), SIO( 3), PIO0( 3), PIO1( 3), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_4_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(RX) , UART1(TX) , I2C0(SDA), PWM0(A), SIO( 4), PIO0( 4), PIO1( 4), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_5_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(CSn), UART1(RX) , I2C0(SCL), PWM0(B), SIO( 5), PIO0( 5), PIO1( 5), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_6_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(SCK), UART1(CTS), I2C1(SDA), PWM1(A), SIO( 6), PIO0( 6), PIO1( 6), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_7_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(TX) , UART1(RTS), I2C1(SCL), PWM1(B), SIO( 7), PIO0( 7), PIO1( 7), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_8_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(RX) , UART1(TX) , I2C0(SDA), PWM0(A), SIO( 8), PIO0( 8), PIO1( 8), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_9_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(CSn), UART1(RX) , I2C0(SCL), PWM0(B), SIO( 9), PIO0( 9), PIO1( 9), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_10_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(SCK), UART1(CTS), I2C1(SDA), PWM1(A), SIO(10), PIO0(10), PIO1(10), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_11_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(TX) , UART1(RTS), I2C1(SCL), PWM1(B), SIO(11), PIO0(11), PIO1(11), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_12_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(RX) , UART0(TX) , I2C0(SDA), PWM0(A), SIO(12), PIO0(12), PIO1(12), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_13_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(CSn), UART0(RX) , I2C0(SCL), PWM0(B), SIO(13), PIO0(13), PIO1(13), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_14_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(SCK), UART0(CTS), I2C1(SDA), PWM1(A), SIO(14), PIO0(14), PIO1(14), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_15_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(TX) , UART0(RTS), I2C1(SCL), PWM1(B), SIO(15), PIO0(15), PIO1(15), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_16_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(RX) , UART0(TX) , I2C0(SDA), PWM0(A), SIO(16), PIO0(16), PIO1(16), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_17_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(CSn), UART0(RX) , I2C0(SCL), PWM0(B), SIO(17), PIO0(17), PIO1(17), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_18_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(SCK), UART0(CTS), I2C1(SDA), PWM1(A), SIO(18), PIO0(18), PIO1(18), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_19_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(TX) , UART0(RTS), I2C1(SCL), PWM1(B), SIO(19), PIO0(19), PIO1(19), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_20_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(RX) , UART1(TX) , I2C0(SDA), PWM0(A), SIO(20), PIO0(20), PIO1(20), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_21_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(CSn), UART1(RX) , I2C0(SCL), PWM0(B), SIO(21), PIO0(21), PIO1(21), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_22_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(SCK), UART1(CTS), I2C1(SDA), PWM1(A), SIO(22), PIO0(22), PIO1(22), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_23_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI0(TX) , UART1(RTS), I2C1(SCL), PWM1(B), SIO(23), PIO0(23), PIO1(23), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_24_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(RX) , UART1(TX) , I2C0(SDA), PWM0(A), SIO(24), PIO0(24), PIO1(24), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_25_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(CSn), UART1(RX) , I2C0(SCL), PWM0(B), SIO(25), PIO0(25), PIO1(25), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_26_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(SCK), UART1(CTS), I2C1(SDA), PWM1(A), SIO(26), PIO0(26), PIO1(26), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_27_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(TX) , UART1(RTS), I2C1(SCL), PWM1(B), SIO(27), PIO0(27), PIO1(27), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_28_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(RX) , UART0(TX) , I2C0(SDA), PWM0(A), SIO(28), PIO0(28), PIO1(28), DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_29_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{DUMMY, SPI1(CSn), UART0(RX) , I2C0(SCL), PWM0(B), SIO(29), PIO0(29), PIO1(29), DUMMY, DUMMY, DUMMY_REMAINDER}

#define GPIO_QSPI_SCK_FUNCS std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(SCK), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(0), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_QSPI_CS_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(SS), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(1), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_QSPI_D0_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(D0), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(2), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_QSPI_D1_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(D1), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(3), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_QSPI_D2_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(D2), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(4), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}
#define GPIO_QSPI_D3_FUNCS  std::array<std::reference_wrapper<GPIOSignal>,32>{SSI(D3), DUMMY, DUMMY, DUMMY, DUMMY, SIO_HI(5), DUMMY, DUMMY, DUMMY, DUMMY, DUMMY_REMAINDER}

#define QSPI_0_NAME SCK
#define QSPI_1_NAME CS
#define QSPI_2_NAME D0
#define QSPI_3_NAME D1
#define QSPI_4_NAME D2
#define QSPI_5_NAME D3

#define CAT2(a, b, c) a##b##c
#define CAT2x(a, b, c) CAT2(a, b, c)
#define EXPAND(x) x
#define GPION_FUNCS(n) EXPAND(GPIO_##n##_FUNCS)
#define QSPI_FUNCS(n) EXPAND(CAT2x(GPIO_QSPI_, QSPI_##n##_NAME, _FUNCS))

#define EVAL_GPIO_BANK0(n) {m_pads_bank0[n], GPION_FUNCS(n)},
#define EVAL_GPIO_QSPI(n) {m_pads_qspi[n], QSPI_FUNCS(n)},
#define EVAL_PAD_BANK0(n) {m_gpio_bank0[n]},
#define EVAL_PAD_QSPI(n) {m_gpio_qspi[n]},
#define EVAL_PAD_BANK0_ADDR(n) {&m_gpio_bank0[n]},
#define EVAL_PAD_QSPI_ADDR(n) {&m_gpio_qspi[n]},
#define EVAL_GPIO_BANK0_N(n) m_gpio_bank0[n],
#define EVAL_GPIO_QSPI_N(n) m_gpio_qspi[n],
#define EVAL_SIO(n) m_sio[n],
#define EVAL_SIO_HI(n) m_sio_hi[n],
#define ARR_REF(type, n) std::array<std::reference_wrapper<type>, n>

RP2040::RP2040::RP2040()
// 2,147,483,647 /2 = 1,073,741,823
// 1,073,741,823 / 500 = 2,147,483
: m_rosc{uint32_t(6'500'000 + (rand()-(RAND_MAX/2))/500)}
, m_pads_bank0{ENUM_32(EVAL_PAD_BANK0)}
, m_pads_qspi{ENUM_6(EVAL_PAD_QSPI)}
, m_gpio_bank0{ENUM_30(EVAL_GPIO_BANK0)}
, m_gpio_qspi{ENUM_6(EVAL_GPIO_QSPI)}
, m_nets{
  {{"GP0"}, {"GP1"}, {"GP2"}, {"GP3"}, {"GP4"}, {"GP5"}, {"GP6"}, {"GP7"}, {"GP8"}, {"GP9"},
  {"GP10"}, {"GP11"}, {"GP12"}, {"GP13"}, {"GP14"}, {"GP15"}, {"GP16"}, {"GP17"}, {"GP18"}, {"GP19"},
  {"GP20"}, {"GP21"}, {"GP22"}, {"GP23"}, {"GP24"}, {"GP25"}, {"GP26"}, {"GP27"}, {"GP28"}, {"GP29"},
  {"SWDIO"}, {"SWCLK"}, {"QSPI_SCK"}, {"QSPI_CS0"}, {"QSPI_SD0"}, {"QSPI_SD1"}, {"QSPI_SD2"}, {"QSPI_SD3"}}
}
, m_sio{{ENUM_30(EVAL_PAD_BANK0_ADDR)}}
, m_sio_hi{{ENUM_6(EVAL_PAD_QSPI_ADDR)}}
, m_clocks{*this}
, m_XIP{m_SSI}
, m_SSI{*this}
, m_ioports{
  {0, m_fifo_01, m_fifo_10, ARR_REF(GPIOSignal, 30){ENUM_30(EVAL_SIO)}, ARR_REF(GPIOSignal, 6){ENUM_6(EVAL_SIO_HI)}}, 
  {1, m_fifo_10, m_fifo_01, ARR_REF(GPIOSignal, 30){ENUM_30(EVAL_SIO)}, ARR_REF(GPIOSignal, 6){ENUM_6(EVAL_SIO_HI)}}}
, m_ROM{load_bootloader(BootloaderVersion::B2)}
, m_bus_masters{m_cores[0].run(), m_cores[1].run()}
, m_ahb_lite{m_DMA}
, m_ahb{*this}
, m_DMA{m_ahb, {
  /*DREQ_PIO0_TX0*/ m_null_dreq,
  /*DREQ_PIO0_TX1*/ m_null_dreq,
  /*DREQ_PIO0_TX2*/ m_null_dreq,
  /*DREQ_PIO0_TX3*/ m_null_dreq,
  /*DREQ_PIO0_RX0*/ m_null_dreq,
  /*DREQ_PIO0_RX1*/ m_null_dreq,
  /*DREQ_PIO0_RX2*/ m_null_dreq,
  /*DREQ_PIO0_RX3*/ m_null_dreq,
  /*DREQ_PIO1_TX0*/ m_null_dreq,
  /*DREQ_PIO1_TX1*/ m_null_dreq,
  /*DREQ_PIO1_TX2*/ m_null_dreq,
  /*DREQ_PIO1_TX3*/ m_null_dreq,
  /*DREQ_PIO1_RX0*/ m_null_dreq,
  /*DREQ_PIO1_RX1*/ m_null_dreq,
  /*DREQ_PIO1_RX2*/ m_null_dreq,
  /*DREQ_PIO1_RX3*/ m_null_dreq,
  /*DREQ_SPI0_TX*/ m_null_dreq,
  /*DREQ_SPI0_RX*/ m_null_dreq,
  /*DREQ_SPI1_TX*/ m_null_dreq,
  /*DREQ_SPI1_RX*/ m_null_dreq,
  /*DREQ_UART0_TX*/ UART0(tx_dreq),
  /*DREQ_UART0_RX*/ UART0(rx_dreq),
  /*DREQ_UART1_TX*/ m_null_dreq,
  /*DREQ_UART1_RX*/ m_null_dreq,
  /*DREQ_PWM_WRAP0*/ m_null_dreq,
  /*DREQ_PWM_WRAP1*/ m_null_dreq,
  /*DREQ_PWM_WRAP2*/ m_null_dreq,
  /*DREQ_PWM_WRAP3*/ m_null_dreq,
  /*DREQ_PWM_WRAP4*/ m_null_dreq,
  /*DREQ_PWM_WRAP5*/ m_null_dreq,
  /*DREQ_PWM_WRAP6*/ m_null_dreq,
  /*DREQ_PWM_WRAP7*/ m_null_dreq,
  /*DREQ_I2C0_TX*/ m_null_dreq,
  /*DREQ_I2C0_RX*/ m_null_dreq,
  /*DREQ_I2C1_TX*/ m_null_dreq,
  /*DREQ_I2C1_RX*/ m_null_dreq,
  /*DREQ_ADC*/ m_null_dreq,
  /*DREQ_XIP_STREAM*/ m_null_dreq,
  /*DREQ_XIP_SSITX*/ m_null_dreq,
  /*DREQ_XIP_SSIRX*/ m_null_dreq,

}}
, m_apb{
    m_resets, 
    m_vreg, 
    m_clocks, 
    m_syscfg, 
    std::array<std::reference_wrapper<GPIO>, 30>{ENUM_30(EVAL_GPIO_BANK0_N)},
    std::array<std::reference_wrapper<GPIO>, 6>{ENUM_6(EVAL_GPIO_QSPI_N)},
    {
      m_interrupts.get_source(IRQ::TIMER_IRQ_0),
      m_interrupts.get_source(IRQ::TIMER_IRQ_1),
      m_interrupts.get_source(IRQ::TIMER_IRQ_2),
      m_interrupts.get_source(IRQ::TIMER_IRQ_3),
    },
    m_interrupts.get_source(IRQ::UART0_IRQ), 
    m_interrupts.get_source(IRQ::UART1_IRQ),
    m_interrupts.get_source(IRQ::SPI0_IRQ), 
    m_interrupts.get_source(IRQ::SPI1_IRQ),
    m_interrupts.get_source(IRQ::I2C0_IRQ),
    m_interrupts.get_source(IRQ::I2C1_IRQ),
    m_interrupts.get_source(IRQ::RTC_IRQ)
  }
, m_core_bus{{m_ioports[0], m_ahb, 0}, {m_ioports[1], m_ahb, 1}}
, m_cores{
  {m_core_bus[0], "core-0", m_cores, m_interrupts}, 
  {m_core_bus[1], "core-1", m_cores, m_interrupts}
}
, m_vcd{"RP2040"}
{
  m_rosc.sink_add(*this);
  clk_sys.sink_add(m_cores[0]);
  clk_sys.sink_add(m_cores[1]);
  clk_sys.sink_add(m_DMA);
  clk_sys.sink_add(m_ahb);
  clk_sys.sink_add(m_XIP);
  clk_sys.sink_add(m_SSI);
  clk_ref.sink_add(m_apb.timer());
  clk_peri.sink_add(m_apb.uart0());
  // clk_per.sink_add(m_apb.uart1());
  clk_sys.sink_add(m_apb.spi0().PCLK());  
  clk_sys.sink_add(m_apb.spi1().PCLK());
  clk_peri.sink_add(m_apb.spi0().SSPCLK());  
  clk_peri.sink_add(m_apb.spi1().SSPCLK());
  clk_rtc.sink_add(m_apb.rtc());

  auto &sim = Simulation::get();

  for (int i = 0; i < 32; i++) {
    m_pads_bank0[i].connect_to_net(&m_nets[i]);
    sim.nets().add_item(m_nets[i].vcd_variable());
  }
  for (int i = 0; i < 6; i++) {
    m_pads_qspi[i].connect_to_net(&m_nets[i + 32]);
    sim.nets().add_item(m_nets[i+32].vcd_variable());
  }
  m_vcd.add_item(m_ahb.vcd());
  m_vcd.add_item(m_cores[0].vcd());
  m_vcd.add_item(m_cores[1].vcd());
  m_vcd.add_item(m_apb.uart0().vcd());
  m_vcd.add_item(m_apb.uart1().vcd());
}

void RP2040::RP2040::reset()
{
  std::cout << "RP2040::reset()" << std::endl;
  m_cores[0].reset();
  m_cores[1].reset();
}

void RP2040::RP2040::run(unsigned int max_ticks)
{
  unsigned int ticks = 0;
  while (ticks++ <= max_ticks) {
    m_tickcnt++;
    clk_sys.tick();
    clk_ref.tick();
    clk_peri.tick();
    clk_rtc.tick();
  }
  m_cores[0].dump();
}
void RP2040::RP2040::tick()
{
  m_tickcnt++;
  clk_sys.tick();
  clk_ref.tick();
  clk_peri.tick();
  clk_rtc.tick();
}

void RP2040::RP2040::load_binary(const std::string &path)
{
  std::cout << "RP2040::load_binary(" << path << ")" << std::endl;
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buf;
  buf.resize(size);

  if (file.read((char*)buf.data(), size))
  {
      /* worked! */
  } else {
      /* failed! */
      std::terminate();
  }

  m_XIP.load_binary_data(buf);
  m_SSI.spidev().load_binary_data(buf);
}

#define READ(wordtype, ctype, out) \
  {\
    ctype out2; \
    if ((op.addr & 0xf000'0000U) == 0xd000'0000U) { \
      m_ioport.read_##wordtype (op.addr, out2); \
    } else { \
      out2 = co_await m_ahb.read_##wordtype (op.addr, m_core_id); \
    }\
    out = out2;}

#define WRITE(wordtype) \
  if ((op.addr & 0xf000'0000U) == 0xd000'0000U) { \
    m_ioport.write_##wordtype (op.addr, op.data); \
  } else { \
    co_await m_ahb.write_##wordtype (op.addr, op.data, m_core_id); \
  }

Task RP2040::RP2040::CoreBus::bus_task()
{
  while(true) {
    auto &op = co_await next_op();
    uint32_t out;
    switch(op.optype) {
      case MemoryOperation::READ_BYTE:
        READ(byte, uint8_t, out);
        op.return_value(out);
        break;
      case MemoryOperation::READ_HALFWORD:
          READ(halfword, uint16_t, out);
          op.return_value(out);
        break;
      case MemoryOperation::READ_WORD:{
        READ(word, uint32_t, out);
        op.return_value(out);
        break;
      }
      case MemoryOperation::WRITE_BYTE:
        WRITE(byte);
        op.return_void();
        break;
      case MemoryOperation::WRITE_HALFWORD:
        WRITE(halfword);
        op.return_void();
        break;
      case MemoryOperation::WRITE_WORD:
        WRITE(word);
        op.return_void();
        break;
    }
  }

}

PortState RP2040::RP2040::IOPort::read_byte(uint32_t addr, uint8_t &out){ 
  uint32_t out2;
  PortState ret = read_word(addr, out2);
  out = out2;
  return ret;
}
PortState RP2040::RP2040::IOPort::read_halfword(uint32_t addr, uint16_t &out){ 
  uint32_t out2;
  PortState ret = read_word(addr, out2);
  out = out2;
  return ret;
}
PortState RP2040::RP2040::IOPort::read_word(uint32_t addr, uint32_t &out){ 
  // std::cout << "IOPort::read_word(" << std::hex << addr << std::dec << ")" << std::endl;
  #define SPINLOCKS_EVAL(num) case (0xd000'0100 + 4*num): out = m_spinlocks.try_lock(num); break;
  out = 0;
  switch(addr) {
    case 0xd000'0000: out = m_cpuid; break;
    case 0xd000'0004: // GPIO_IN
    {
      for (int i = 0; i < 30; i++) 
        out |= m_sio[i].get().get_input() << i; 
      break;
    }
    case 0xd000'0008: // force CS high to enable flash boot
    {
      for (int i = 0; i < 6; i++) 
        out |= m_sio_hi[i].get().get_input() << i; 
      break;
    }
    case 0xd000'0050: out = m_tx_fifo.status_send() | m_rx_fifo.status_recv(); break;
    // case 0xd000'0054: out = ; break;
    case 0xd000'0058: out = m_rx_fifo.recv(); break;
    case 0xd000'0060: out = m_divider.get_dividend(); break;
    case 0xd000'0064: out = m_divider.get_divisor(); break;
    case 0xd000'0068: out = m_divider.get_dividend(); break;
    case 0xd000'006c: out = m_divider.get_divisor(); break;
    case 0xd000'0070: out = m_divider.get_quotient(); break;
    case 0xd000'0074: out = m_divider.get_remainder(); break;
    case 0xd000'0078: out = m_divider.get_status(); break;
    ENUM_SPINLOCKS(SPINLOCKS_EVAL)
    default: throw ARMv6M::BusFault{addr};
  }
  #undef SPINLOCKS_EVAL
  return PortState::SUCCESS; 
}
PortState RP2040::RP2040::IOPort::write_byte(uint32_t addr, uint8_t in){ throw ARMv6M::BusFault(addr); }
PortState RP2040::RP2040::IOPort::write_halfword(uint32_t addr, uint16_t in){ throw ARMv6M::BusFault(addr); }
PortState RP2040::RP2040::IOPort::write_word(uint32_t addr, uint32_t in){ 
  std::cout << "IOPort::write_word(" << std::hex << addr << ", " << in << std::dec << ")" << std::endl;
  #define SPINLOCKS_EVAL(num) case (0xd000'0100 + 4*num): m_spinlocks.unlock(num); break;
  switch(addr) {
    case 0xd000'0000: break;
    case 0xd000'0010: {
      for (int i = 0; i < 30; i++) 
        m_sio[i].get().set_output(in & (1<<i)); 
      break;
    }
    case 0xd000'0014: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_output(true); 
      break;
    }
    case 0xd000'0018: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_output(false); 
      break;
    }
    case 0xd000'001c: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_output(!m_sio[i].get().get_output());
      break;
    }
    case 0xd000'0020: {
      for (int i = 0; i < 30; i++) 
        m_sio[i].get().set_oe(in & (1<<i)); 
      break;
    }
    case 0xd000'0024: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_oe(true); 
      break;
    }
    case 0xd000'0028: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_oe(false); 
      break;
    }
    case 0xd000'0030: {
      for (int i = 0; i < 30; i++) 
        if (in & (1<<i)) m_sio[i].get().set_oe(!m_sio[i].get().get_output());
      break;
    }
    case 0xd000'0050: 
      if (in & (1<<2)) m_tx_fifo.clear_send();
      if (in & (1<<3)) m_rx_fifo.clear_recv();
      break;
    case 0xd000'0054: m_tx_fifo.send(in); break;
    case 0xd000'0060: m_divider.set_udividend(in); break;
    case 0xd000'0064: m_divider.set_udivisor(in); break;
    case 0xd000'0068: m_divider.set_sdividend(in); break;
    case 0xd000'006c: m_divider.set_sdivisor(in); break;
    case 0xd000'0070: m_divider.set_quotient(in); break;
    case 0xd000'0074: m_divider.set_remainder(in); break;
    ENUM_SPINLOCKS(SPINLOCKS_EVAL)

    default: throw ARMv6M::BusFault{addr};
  }
  #undef SPINLOCKS_EVAL
  return PortState::SUCCESS; 
}