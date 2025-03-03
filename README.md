# PicoSimulator

PicoSimulator (AKA PicoSim) is designed to be a cycle-accurate full-system emulator for the RP2040/Raspberry Pi Pico and other similar boards. It features (WIP) full crossbar bus fabric simulation with wait-states and stalls and once complete will hopefully be indistinguishable from real hardware to software.

## Usage

PicoSim is built as both a library which contains the simulation logic, and an application which parses command line arguments and runs the simulation.

## Performance considerations

Most of the performance critical code is going to include emulating the ARMv6M Core and memory accesses to SRAM/XIP as this is where most bus transfers will occur.

Peripheral configuration is likely to be a one-off event in execution and will be allowed to take longer, however accesses to peripheral registers should be as fast as possible in the majority of cases. IO reads/writes are likely to be the slowest component of all.

## Progress

- [ ] RP2040 Processor
  - [ ] Clocks
    - [ ] PLLs
    - [x] Clock MUXes
    - [x] Clock Dividers
  - [x] ARMv6M Core
  - [x] Bus Fabric
    - [x] AHB Crossbar
    - [x] AHB-Lite Splitter
    - [x] APB Bridge+splitter
  - [ ] IOPort SIO
    - [x] CPUID
    - [x] FIFOs
    - [ ] Spinlocks
    - [ ] Integer Divider
    - [ ] Interpolator
    - [x] SIO GPIO
  - [x] Interrupts
    - [x] NVIC
  - [x] Event Signals
  - [x] DMA
  - [ ] Peripherals
    - [x] UART
    - [x] SPI
    - [ ] I2C
    - [ ] PWM
    - [ ] ADC
    - [ ] PIO
  <!-- - [ ]  -->

## Peripherals

### UART

The UART is currently functional enough to provide output from the emulator. You can pass either a file or a PTY using `--uart0=/path/to/file`. This will either write the output of the UART to file or to a PTY of which the other end can be opened by minicom or similar serial port software.
A PTY pair can be spun up using `socat -d -d pty,rawer,echo=0[,link=/symlink/file0] pty,rawer,echo=0[,link=/symlink/file1]`. Using symlinks allows the path to be consistent, as socat may create pty with any number.

### SPI

SPI is mostly functional. External devices can be connected to either hardware SPI interface or bitbanged pins. These external devices can be built-in or loaded from shared libraries.

### I2C

### PIO

### Timers

### PWM

## Device Loading

Devices can be loaded using shared-object/dll type libraries. These provide a standard factory interface which takes a device name and returns a unique_ptr to `IODevice`. This class is responsible for encapsulating devices with Pins or Connections to the real world. These `NetConnection`s are given names and can be looked up to connect to other Nets. In the future I hope to add metadata to these connections allowing the Net simulation to be bypassed entirely in favour of function pointer handling.

## Tracing

PicoSim can be built with Tracing enabled. This allows complete visibility of the internal state of the emulator through time, along with the state of pins and connections allowing simplified debugging of devices. Performance is significantly degraded when this is enabled so this feature is provid as a separate build. Individual groups or Modules can be enabled/disabled as needed, disabling unwanted signals will speed up emulation.