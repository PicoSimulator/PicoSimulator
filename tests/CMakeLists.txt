include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(picosim_tests
  picosim_tests.cpp
)

target_link_libraries(
  picosim_tests
  picosim
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(picosim_tests)


include(ExternalProject)

ExternalProject_Add(
  rp2040_tests
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/embed/rp2040
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/embed/rp2040
  STAMP_DIR ${CMAKE_CURRENT_BINARY_DIR}/embed/rp2040

  CMAKE_ARGS -DPICO_SDK_FETCH_FROM_GIT=ON
  INSTALL_COMMAND cmake -E echo "Skipping install step."
  CONFIGURE_COMMAND "cmake" "${CMAKE_CURRENT_LIST_DIR}/embed/rp2040"
  BUILD_ALWAYS 1
)