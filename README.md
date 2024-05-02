# PicoSim

Picosim is designed to be a cycle-accurate full-system emulator for the RP2040/Raspberry Pi Pico and other similar boards. It features (WIP) full crossbar bus fabric simulation with wait-states and stalls and once complete will hopefully be indistinguishable from real hardware to software.

## Performance considerations

Most of the performance critical code is going to include emulating the ARMv6M Core and memory accesses to SRAM/XIP as this is where most bus transfers will occur.

Peripheral configuration is likely to be a one-off event in execution and will be allowed to take longer, however accesses to peripheral registers should be as fast as possible in the majority of cases. IO reads/writes are likely to be the slowest component of all.

## Subsystems

### UART

The UART is currently functional enough to provide output from the emulator. You can pass either a file or a PTY using `--uart0=/path/to/file`. This will either write the output of the UART to file or to a PTY of which the other end can be opened by minicom or similar serial port software.
A PTY pair can be spun up using `socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`