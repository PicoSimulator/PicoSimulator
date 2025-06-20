cmake_minimum_required(VERSION 3.22)
project(PicoSim)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-ggdb -g -std=c++20)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
# using Clang
  add_compile_options(-gdwarf-4)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # using GCC
  add_compile_options(-fcoroutines)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-ggdb)
    add_link_options(-ggdb)
  else()
    add_compile_options(-O3 -flto)
    add_link_options(-ggdb)
    add_link_options(-flto)
    endif()
endif()

set(TARGET picosim)
add_library(${TARGET} STATIC)
if (DEFINED CACHE{TRACE_ENABLED})
  target_compile_definitions(${TARGET} PRIVATE
    config_VCD_TRACE_ENABLED=$CACHE{TRACE_ENABLED}
  )
endif()
target_compile_options(${TARGET} PRIVATE 
  -std=c++20 
  -fcoroutines
  -fPIC
)
target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include/)
add_subdirectory(src)

add_subdirectory(app)
add_subdirectory(extlib)
# add_subdirectory(tests)