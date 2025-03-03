target_sources(${TARGET} PRIVATE
  clock.cpp
  memory_device.cpp
  simulation.cpp
  tracing/vcd.cpp
  common/gpio.cpp
  arch/arm/armv6m/core.cpp
  arch/arm/armv6m/mpu.cpp
  platform/rpi/rp2040/bootloader.cpp
  platform/rpi/rp2040/rp2040.cpp
  platform/rpi/rp2040/bus/apb.cpp
  platform/rpi/rp2040/bus/ahb.cpp
  platform/rpi/rp2040/bus/ahb_lite.cpp
  platform/rpi/rp2040/peri/xip.cpp
  platform/rpi/rp2040/peri/ssi.cpp
  platform/rpi/rp2040/peri/clocks.cpp
  platform/rpi/rp2040/peri/dma/dma.cpp
  platform/rpi/rp2040/peri/dma/dreq.cpp
  platform/rpi/rp2040/pad.cpp
  platform/rpi/rp2040/gpio.cpp
  ext/io/net.cpp
)

target_include_directories(${TARGET} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)