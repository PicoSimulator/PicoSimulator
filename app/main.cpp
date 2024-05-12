#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rp2040.hpp"
#include "argparse/argparse.hpp"

struct MyArgs : public argparse::Args {
  std::string &binary = arg("flash_binary", "Path to binary file to load into flash");
  std::optional<std::string> &uart0 = kwarg("uart0", "Path to UART0 file, can be PTY");
  int &max_ticks = kwarg("max_ticks", "Number of ticks to run").set_default(std::numeric_limits<int>::max());
};

RP2040::RP2040 g_rp2040{};

void my_handler(int s){
  std::cerr << "EXITING" << std::endl;
  exit(1); 
}

int main(int argc, char** argv)
{
  auto args = argparse::parse<MyArgs>(argc, argv);
  args.print();
  g_rp2040.load_binary(args.binary);
  if (args.uart0) {
    std::cout << "test" << std::endl;
    g_rp2040.UART0().open(*args.uart0);
  }
  
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  g_rp2040.reset();
  g_rp2040.run(args.max_ticks);
}