cmake_minimum_required(VERSION 3.22)
project(PicoSim)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-ggdb -g -O3 -std=c++20)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
# using Clang
  add_compile_options(-gdwarf-4)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # using GCC
  add_compile_options(-flto -fcoroutines)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-ggdb)
    add_link_options(-ggdb)
  else()
    add_compile_options(-O3)
    add_link_options(-ggdb)
    endif()
  add_link_options(-flto)
endif()

set(TARGET picosim)
add_library(${TARGET} STATIC)
target_compile_options(${TARGET} PRIVATE -std=c++20 -fcoroutines)
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include/)
add_subdirectory(src)

add_subdirectory(app)
add_subdirectory(tests)