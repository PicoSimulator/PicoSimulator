target_sources(${TARGET} PRIVATE
  clock.cpp
  rp2040.cpp
  rp2040_bootloader.cpp
  memory_device.cpp
  armv6m/core.cpp
  armv6m/mpu.cpp
  rp2040/bus/apb.cpp
  rp2040/bus/ahb.cpp
  rp2040/bus/ahb_lite.cpp
  rp2040/peri/xip.cpp
  rp2040/peri/ssi.cpp
  rp2040/peri/dma/dma.cpp
  rp2040/pad.cpp
  ext/io/net.cpp
)

target_include_directories(${TARGET} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)