#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rp2040.hpp"
#include "simulation.hpp"
#include "argparse/argparse.hpp"
#include <chrono>
#include "ext/io/pulls.hpp"

struct MyArgs : public argparse::Args {
  std::string &binary = arg("flash_binary", "Path to binary file to load into flash");
  std::optional<std::string> &uart0 = kwarg("uart0", "Path to UART0 file, can be PTY");
  unsigned int &max_ticks = kwarg("max_ticks", "Number of ticks to run").set_default(std::numeric_limits<unsigned int>::max());
  bool &core0_trace = kwarg("core0.trace", "Enable trace for core 0").set_default(false);
  bool &core1_trace = kwarg("core1.trace", "Enable trace for core 1").set_default(false);
  bool &core1_enable = kwarg("core1.enable", "Enable core 1").set_default(false);
};

struct Args : public argparse::Args {
  std::string &command = arg("command", "Command to run");
};

RP2040::RP2040 g_rp2040{};
std::chrono::time_point<std::chrono::high_resolution_clock> g_start_time;

void simulation_complete(){
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time-g_start_time;
  std::cerr << "Run for " << std::dec << g_rp2040.tickcnt() << " ticks" << std::endl;
  std::cerr << "Run for " << elapsed_seconds.count() << " seconds" << std::endl;
  std::cerr << "Emulation speed " << g_rp2040.tickcnt()/elapsed_seconds.count()/1000'000.0 << " MHz" << std::endl;
  std::cerr << "EXITING" << std::endl;
  auto &sim = Simulation::get();
  sim.abort();
}
void my_handler(int s){
  simulation_complete();
}

//usage
// picosim [command] [options]
// commands:
//   help - Print this help
//   run [options] - Run the simulation
//   list <configs|boards>
//   new <project> - Create a new project
// run options:
//   <config name> - 
//   --upload <binary> - Upload the binary to the target
//   -c, --config <config> - Use the specified configuration

int main(int argc, char** argv)
{
  auto args = argparse::parse<MyArgs>(argc, argv);
  args.print();
  g_rp2040.load_binary(args.binary);
  auto &cs_net = g_rp2040.net("QSPI_CS0");
  g_rp2040.SSI().spidev().cs_pin().connect_to_net(&cs_net);
  // we need a pullup on CS in order to
  // pass flash boot test
  PullUp cs_pullup;
  cs_pullup.connect_to_net(&cs_net);
  if (args.uart0) {
    g_rp2040.UART0().open(*args.uart0);
  }
  
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  g_start_time = std::chrono::high_resolution_clock::now();
  g_rp2040.reset();
  g_rp2040.core0().set_trace_enabled(args.core0_trace);
  g_rp2040.core1().set_trace_enabled(args.core1_trace);

  auto &sim = Simulation::get();
  sim.vcd().top().add_item(g_rp2040.vcd());
  sim.vcd().open("rp2040.vcd");
  sim.vcd().write_header();
  
  sim.run(Simulation::from_seconds(60));
  simulation_complete();
}