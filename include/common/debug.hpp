#pragma once

#if DEBUG
#include <iostream>
#define debug_stream(stream) (stream)
#else
#include <fstream>
#define debug_stream(stream) (std::ostream{nullptr})
#endif

#define debug_log() debug(std::cout)
#define debug_warn() debug(std::cerr)
#define debug_err() debug(std::cerr)
#define debug_crit() debug(std::cerr)

#define DEBUG_LEVEL_LOG 0
#define DEBUG_LEVEL_WARN 1
#define DEBUG_LEVEL_ERR 2
#define DEBUG_LEVEL_CRIT 3

#define debug_0() debug_log
#define debug_1() debug_warn
#define debug_2() debug_err
#define debug_3() debug_crit

#define CAT(a, b) a##b

#define debug(level) CAT(debug_, level)()