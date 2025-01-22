#include "simulation.hpp"

Simulation &Simulation::get(){
  static Simulation sim;
  return sim;
}