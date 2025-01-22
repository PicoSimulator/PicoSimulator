#pragma once

#include "clock.hpp"
#include "simulation.hpp"

class ROsc : public Clock, public IClockable{
public:
  ROsc(uint32_t freq)
  {
    auto &sim = Simulation::get();
    sim.schedule_periodic(Simulation::from_hz(freq), *this);
  }
  void tick() override 
  {
    do_tick();
  }
  void tock() override
  {
    do_tock();
  }
protected:
private:


};