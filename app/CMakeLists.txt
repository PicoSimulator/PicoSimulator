add_subdirectory(argparse)

add_executable(picosim_app)
target_sources(picosim_app PRIVATE main.cpp)
target_link_libraries(picosim_app picosim argparse)