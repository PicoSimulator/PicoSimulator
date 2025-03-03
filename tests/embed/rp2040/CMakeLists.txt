cmake_minimum_required(VERSION 3.22)
set(PICO_SDK_FETCH_FROM_GIT ON)
include(./pico_sdk_import.cmake)
pico_sdk_init()
project("RP2040 Tests" LANGUAGES C CXX)


add_executable(rp2040_tests
  tests.cpp
  ../../util/sha256.c
)

target_include_directories(rp2040_tests PRIVATE ../../util/)

target_link_libraries(rp2040_tests PRIVATE pico_stdlib pico_printf)

pico_add_extra_outputs(rp2040_tests)