#include "platform/rpi/rp2040/peri/clocks.hpp" 
#include "platform/rpi/rp2040/rp2040.hpp"

ClockDiv &Clocks::clk_gpout0() { return m_rp2040.clk_gpout0(); }
