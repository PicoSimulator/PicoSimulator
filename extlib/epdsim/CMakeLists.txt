include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# Find all source files in a single current directory
# Save the name to DIR_Config_SRCS
aux_source_directory(. DIR_epdsim_SRCS)

# Generate the link library
add_library(epdsim SHARED ${DIR_epdsim_SRCS})

target_link_libraries(epdsim PRIVATE picosim sfml-graphics sfml-window sfml-system X11)