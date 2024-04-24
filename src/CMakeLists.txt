target_sources(${TARGET} PRIVATE
  clock.cpp
  rp2040.cpp
  rp2040_bootloader.cpp
  armv6m/core.cpp
  armv6m/mpu.cpp
  bus/apb.cpp
)

target_include_directories(${TARGET} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)