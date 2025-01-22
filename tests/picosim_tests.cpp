#include <gtest/gtest.h>

#include "clock.hpp"

#include <fstream>
#include <format>
#include <iomanip>
#include <bitset>

class ClockCounter : public IClockable {
public:
  ClockCounter() : m_tick_count{0}, m_tock_count{0} {}
  void tick() override { m_tick_count++; }
  void tock() override { m_tock_count++; }
  uint32_t count() const { return m_tick_count+m_tock_count; }
  uint32_t tick_count() const { return m_tick_count; }
  uint32_t tock_count() const { return m_tock_count; }
  void reset() { m_tick_count = 0; m_tock_count = 0; }
protected:
private:
  uint32_t m_tick_count, m_tock_count;
};

TEST(ClockTests, Simple_clock_divider_tick_test) {
  SimpleClockDiv clock;
  ClockCounter counter;
  clock.sink_add(counter);
  clock.divisor(2);

  for(int i = 0; i < 10; i++) {
    clock.tick();
  }
  EXPECT_EQ(counter.count(), 5);
  EXPECT_EQ(counter.tick_count(), 5);
  EXPECT_EQ(counter.tock_count(), 0);
}

TEST(ClockTests, Simple_clock_divider_tick_tock_test) {
  SimpleClockDiv clock;
  ClockCounter counter;
  clock.sink_add(counter);
  clock.divisor(2);

  for(int i = 0; i < 10; i++) {
    clock.tick();
    clock.tock();
  }
  EXPECT_EQ(counter.count(), 5);
  EXPECT_EQ(counter.tick_count(), 5);
  EXPECT_EQ(counter.tock_count(), 0);
}


TEST(ClockTests, Clock_divider_tick_test) {
  // ClockDiv clock;
  // ClockCounter counter;
  // clock.sink_add(counter);
  // clock.divisor(2);

  // for(int i = 0; i < 10; i++) {
  //   clock.tick();
  // }
  // EXPECT_EQ(counter.count(), 10);
  // EXPECT_EQ(counter.tick_count(), 5);
  // EXPECT_EQ(counter.tock_count(), 5);
  SimpleClockDiv clock;
  ClockDiv cdivs[10];
  ClockCounter counters[10];
  clock.divisor(2);
  for (int i = 0; i < 10; i++) {
    clock.sink_add(cdivs[i]);
    cdivs[i].sink_add(counters[i]);
    cdivs[i].divisor(i+2);
  }

  // divider 2
  // 7 cycles -> 4 ticks, 3 tocks
  // 8 cycles -> 4 ticks, 4 tocks
  // 9 cycles -> 5 ticks, 4 tocks
  // 10 cycles -> 5 ticks, 5 tocks
  // 11 cycles -> 6 ticks, 5 tocks
  // 12 cycles -> 6 ticks, 6 tocks
  // 13 cycles -> 7 ticks, 6 tocks
  // ticks = (cycles + divisor - 1) / divisor
  // tocks = (cycles - divisor + 1) / divisor
  // divider 3
  // 1 cycles -> 1 ticks
  // 2 cycles -> 1 ticks, 1 tocks
  // 3 cycles -> 1 ticks, 1 tocks
  // 4 cycles -> 2 ticks, 1 tocks
  // 5 cycles -> 2 ticks, 2 tocks
  // 6 cycles -> 2 ticks, 2 tocks
  // 7 cycles -> 3 ticks, 2 tocks
  // ticks = (cycles + divisor - 1) / divisor
  // tocks = (cycles - divisor + 1) / divisor
  // divider 5
  // 1 cycles -> 1 ticks
  // 2 cycles -> 1 ticks, 0 tocks
  // 3 cycles -> 1 ticks, 1 tocks
  // 4 cycles -> 1 ticks, 1 tocks
  // 5 cycles -> 1 ticks, 1 tocks
  // 6 cycles -> 2 ticks, 1 tocks
  // ticks = (cycles-1) / divisor + 1 = (cycles + divisor - 1) / divisor
  std::string filename = "Clock_divider_tick_test.vcd";
  std::ofstream out(filename);
  out << "$timescale 1ns $end" << std::endl;
  out << "$scope module clock $end" << std::endl;
  // out << "$var wire 1 # clock $end" << std::endl;
  for (int i = 0; i < 10; i++) {
    out << "$var wire 1 $" << i << " div_" << i+2 << " $end" << std::endl;
    out << "$var real 32 !" << i << " div_" << i+2 << "_count $end" << std::endl;
    out << "$var real 32 %" << i << " div_" << i+2 << "_tick_count $end" << std::endl;
    out << "$var real 32 @" << i << " div_" << i+2 << "_tock_count $end" << std::endl;
  }
  out << "$upscope $end" << std::endl;
  out << "$enddefinitions $end" << std::endl;
  out << "$dumpvars" << std::endl;
  auto equal = [](auto a, auto b, ...) { return a == b; };

  for(int j = 0; j < 10; j++) {
    auto &counter = counters[j];
    int divisor = j+2;
    // out << (cdivs[j].state()?"1":"0") << "$" << j << std::endl;
    // out << "b0!" << j << std::endl;
  }
  out << "#0" << std::endl;

  int max_ticks = 50;
  for (int i = 0; i < max_ticks; i++) {
    
    clock.tick();
    // out << (clock.state()?"1":"0") << "#" << std::endl;
    for (int j = 0; j < 10; j++) {
      auto &counter = counters[j];
      int divisor = j+2;
      out << (cdivs[j].state()?"1":"0") << "$" << j << std::endl;
      out << "r" << counter.count() << " !" << j << std::endl;
      out << "r" << counter.tick_count() << " %" << j << std::endl;
      out << "r" << counter.tock_count() << " @" << j << std::endl;
      // EXPECT_EQ(counter.count(), ((i+1)+divisor-1)/divisor);
      EXPECT_PRED4(equal, counter.tick_count(), ((i)/2+divisor)/divisor, (i+1), divisor);
      // EXPECT_PRED4(equal, counter.tock_count(), ((i+divisor)/2)/divisor, (i+1), divisor);
    }
    out << "#" << i+1 << std::endl;
  }
  
}

TEST(ClockTests, Clock_divider_tick_tock_test) {
  ClockDiv clock;
  ClockDiv cdivs[10];
  ClockCounter counters[10];
  clock.divisor(2);
  for (int i = 0; i < 10; i++) {
    clock.sink_add(cdivs[i]);
    cdivs[i].sink_add(counters[i]);
    cdivs[i].divisor(i+2);
  }

  // divider 2
  // 7 cycles -> 4 ticks, 3 tocks
  // 8 cycles -> 4 ticks, 4 tocks
  // 9 cycles -> 5 ticks, 4 tocks
  // 10 cycles -> 5 ticks, 5 tocks
  // 11 cycles -> 6 ticks, 5 tocks
  // 12 cycles -> 6 ticks, 6 tocks
  // 13 cycles -> 7 ticks, 6 tocks
  // ticks = (cycles + divisor - 1) / divisor
  // tocks = (cycles - divisor + 1) / divisor
  // divider 3
  // 1 cycles -> 1 ticks
  // 2 cycles -> 1 ticks, 1 tocks
  // 3 cycles -> 1 ticks, 1 tocks
  // 4 cycles -> 2 ticks, 1 tocks
  // 5 cycles -> 2 ticks, 2 tocks
  // 6 cycles -> 2 ticks, 2 tocks
  // 7 cycles -> 3 ticks, 2 tocks
  // ticks = (cycles + divisor - 1) / divisor
  // tocks = (cycles - divisor + 1) / divisor
  // divider 5
  // 1 cycles -> 1 ticks
  // 2 cycles -> 1 ticks, 0 tocks
  // 3 cycles -> 1 ticks, 1 tocks
  // 4 cycles -> 1 ticks, 1 tocks
  // 5 cycles -> 1 ticks, 1 tocks
  // 6 cycles -> 2 ticks, 1 tocks
  // ticks = (cycles-1) / divisor + 1 = (cycles + divisor - 1) / divisor
  std::string filename = "Clock_divider_tick_tock_test.vcd";
  std::ofstream out(filename);
  out << "$timescale 1ns $end" << std::endl;
  out << "$scope module clock $end" << std::endl;
  out << "$var wire 1 # clock $end" << std::endl;
  for (int i = 0; i < 10; i++) {
    out << "$var wire 1 $" << i << " div_" << i+2 << " $end" << std::endl;
    out << "$var real 32 !" << i << " div_" << i+2 << "_count $end" << std::endl;
    out << "$var real 32 %" << i << " div_" << i+2 << "_tick_count $end" << std::endl;
    out << "$var real 32 @" << i << " div_" << i+2 << "_tock_count $end" << std::endl;
  }
  out << "$upscope $end" << std::endl;
  out << "$enddefinitions $end" << std::endl;
  out << "$dumpvars" << std::endl;
  auto equal = [](auto a, auto b, ...) { return a == b; };

  for(int j = 0; j < 10; j++) {
    auto &counter = counters[j];
    int divisor = j+2;
    // out << (cdivs[j].state()?"1":"0") << "$" << j << std::endl;
    // out << "b0!" << j << std::endl;
  }
  out << "#0" << std::endl;

  int max_ticks = 30;
  for (int i = 0; i < max_ticks; i++) {
    
    clock.tick();
    out << (clock.state()?"1":"0") << "#" << std::endl;
    for (int j = 0; j < 10; j++) {
      auto &counter = counters[j];
      int divisor = j+2;
      out << (cdivs[j].state()?"1":"0") << "$" << j << std::endl;
      out << "r" << counter.count() << " !" << j << std::endl;
      out << "r" << counter.tick_count() << " %" << j << std::endl;
      out << "r" << counter.tock_count() << " @" << j << std::endl;
      EXPECT_EQ(counter.count(), ((i+1)+divisor-1)/divisor);
      EXPECT_PRED4(equal, counter.tick_count(), ((i)/2+divisor)/divisor, (i+1), divisor);
      EXPECT_PRED4(equal, counter.tock_count(), ((i+divisor)/2)/divisor, (i+1), divisor);
    }
    out << "#" << i+1 << std::endl;
  }
}
