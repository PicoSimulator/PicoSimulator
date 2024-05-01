#include "rp2040.hpp"
#include "argparse/argparse.hpp"

struct MyArgs : public argparse::Args {
  std::string &binary = arg("flash binary", "Path to binary file to load into flash");
};

RP2040::RP2040 g_rp2040{};

int main(int argc, char** argv)
{
  auto args = argparse::parse<MyArgs>(argc, argv);
  g_rp2040.load_binary(args.binary);

  g_rp2040.reset();
  g_rp2040.run();
}