add_subdirectory(argparse)
add_subdirectory(nlohmann_json)

add_executable(picosim_app)
target_sources(picosim_app PRIVATE 
  main.cpp 
  loader.cpp
  schema.cpp
)
target_link_libraries(picosim_app PRIVATE
  picosim 
  argparse 
  nlohmann_json::nlohmann_json
)