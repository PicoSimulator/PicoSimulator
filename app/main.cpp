#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "simulation.hpp"
#include "argparse/argparse.hpp"
#include <chrono>
#include "loader.hpp"
#include "schema.hpp"
#include <filesystem>

namespace fs = std::filesystem;


std::chrono::time_point<std::chrono::high_resolution_clock> g_start_time;

void simulation_complete(){
  static bool complete = false;
  if(complete) return;
  complete = true;
  auto end_time = std::chrono::high_resolution_clock::now();
  auto &sim = Simulation::get();
  std::chrono::duration<double> elapsed_seconds = end_time-g_start_time;
  auto &stream = std::cerr;
  stream << "Simulation complete\n";
  stream << "Run for                : " << std::dec << sim.current_time() << " simulation ticks\n";
  stream << "Run for                : " << elapsed_seconds.count() << " seconds\n";
  stream << "Emulated               : " << sim.to_microseconds(sim.current_time()) / 1'000'000.0 << " Seconds\n";
  stream << "Emulation speed        : " << sim.current_time() / elapsed_seconds.count() / sim.from_seconds(1) * 100.0 << "% real time\n";
  stream << "EXITING" << std::endl;
  sim.abort();
}
void my_handler(int s){
  simulation_complete();
}


fs::path get_user_appdata_path(){
#ifdef _WIN32
  return fs::path{std::getenv("APPDATA")};
#else
  return fs::path{std::getenv("HOME")}/".config";
#endif
}

fs::path get_config_path(){

  return get_user_appdata_path()/"picosim/config.json";
}


json get_config(){
  auto config_path = get_config_path();
  if (!fs::exists(config_path)) {
    fs::create_directories(config_path.parent_path());
    std::ofstream file{config_path};
    json config = {
      {"environments", json::array()},
      {"libraries", json::array()},
    };
    file << config.dump(2);
    file.close();
  }
  std::ifstream file(config_path);
  return json::parse(file);
}

void save_config(const json &config){
  auto config_path = get_config_path();
  std::ofstream file(config_path);
  file << config.dump(2);
  file.close();
}

fs::path get_env_path(const std::string &name){
  auto config = get_config();
  for (auto &env : config["environments"]) {
    if (env["name"] == name) {
      return env["path"];
    }
  }
  // check if this is a path to an environment
  if(fs::exists(name)){
    return name;
  }
  return {};  
}

json get_env(const std::string &name){
  fs::path env_path = get_env_path(name);
  if (env_path.empty())
    throw std::runtime_error("Environment not found");
  std::ifstream file(env_path);
  return json::parse(file);
}

void save_env(const std::string &name, const json &env){
  auto config = get_config();
  auto env_path = get_config_path().parent_path()/name;
  std::ofstream file(env_path/"env.json");
  file << env.dump(2);
  file.close();
  auto &envs = config["environments"];
  json cfg_env = {
    {"name", name},
    {"path", env_path/"env.json"}
  };
  for (auto &env : config["environments"]) {
    if (env["name"] == name) {
      env = cfg_env;
      save_config(config);
      return;
    }
  }
  config["environments"].push_back(cfg_env);
  save_config(config);
}

struct RunArgs : public argparse::Args {
  std::string &config = arg("config", "Configuration to run");
  // std::optional<std::string> &upload = kwarg("upload", "Binary to upload to target");
  unsigned long long &max_ticks = kwarg("max_ticks", "Number of ticks to run").set_default(std::numeric_limits<unsigned long long>::max());
  std::vector<std::string> &args = kwarg("p,params", "Override environment parameters").set_default(std::vector<std::string>{});
  int run() override{
    fs::path env_path = get_env_path(config);
    auto env_config = get_env(env_path);
    auto &sim = Simulation::get();
    std::map<std::string, std::unique_ptr<IODevice>> &devices = sim.components();
    std::map<std::string, std::unique_ptr<Net>> nets;
    fs::current_path(env_path.parent_path());
    // instantiate components
    for (auto &component : env_config["components"]) {
      std::string name = component["name"];
      auto dev = create_device(component["device"]);
      devices[name] = std::move(dev);
    }
    // apply component params
    // must be done after all are instantiated as they may
    // refer back to each other
    for (auto &param : args) {
      std::string name = param.substr(0, param.find('='));
      std::string value = param.substr(param.find('=') + 1);
      std::string component_name = name.substr(0, name.find('.'));
      json *component = nullptr;
      for (auto &_component : env_config["components"]) {
        if (_component["name"] == component_name) {
          component = &_component;
          break;
        }
      }
      if (component == nullptr) {
        throw std::runtime_error("Component not found: " + component_name);
      }
      if (!component->contains("params")) {
        (*component)["params"] = json::object();
      }
      std::string param_name = name.substr(name.find('.') + 1);
      (*component)["params"][param_name] = value;
    }
    for (auto &component : env_config["components"]) {
      std::string name = component["name"].template get<std::string>();
      auto &dev = *devices[name];
      for (auto &[param, value] : component["params"].items()) {
        if (!dev.set_param(param, value)) {
          throw std::runtime_error("Failed to set param: " + param + " on " + name);
        }
      }
    }

    // create Nets and connections.
    for (auto &net : env_config["nets"]) {
      std::string name = net["name"];
      nets[name] = std::make_unique<Net>(name);
      auto *net_obj = nets[name].get();
      for (auto &conn : net["connections"]) {
        std::string conn_name = conn.template get<std::string>();
        if (conn_name.find('.') != std::string::npos) {
          std::string left = conn_name.substr(0, conn_name.find('.'));
          std::string right = conn_name.substr(conn_name.find('.') + 1);
          if (!devices.contains(left)) {
            throw std::runtime_error("Device not found: " + left);
          }
          auto &dev = devices[left];
          auto *pin = dev->get_named_pin(right, false);
          if (!pin) {
            throw std::runtime_error("Pin not found: " + right + " on " + left);
          }
          pin->connect_to_net(net_obj);
        } else {
          auto *pin = NetConnection::special(conn_name);
          if (!pin) {
            throw std::runtime_error("Special pin not found: " + conn_name);
          }
          pin->connect_to_net(net_obj);
        }
      }
      sim.nets().add_item(net_obj->vcd_variable());
    }

    
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    
    g_start_time = std::chrono::high_resolution_clock::now();
  
    sim.run(max_ticks);
    sim.exit();

    return 0;
  }
};

struct ListArgs : public argparse::Args {
  bool &envs = flag("e,envs", "List available environments");
  bool &boards = flag("b,boards", "List available boards");
  bool &libs = flag("l,libs", "List available libraries");
  int run() override{
    std::reference_wrapper<bool> args[] = {envs, boards};
    bool any = false;
    for(const bool &arg : args){
      if(arg) any = true;
    }
    if(!any){
      for (bool &arg : args){
        arg = true;
      }
    }


    auto config = get_config();
    if (envs) {
      std::cout << "Environments:" << std::endl;
      for (auto &env : config["environments"]) {
        std::cout << "  " << std::string{env["name"]} << std::endl;
        std::cout << "    " << std::string{env["path"]} << std::endl;
      }
    }
    if (boards) {
      std::cout << "Boards:" << std::endl;
      for (auto &board : config["boards"]) {
        std::cout << "  " << std::string{board["name"]} << std::endl;
        std::cout << "    " << std::string{board["path"]} << std::endl;
      }
    }
    if (libs) {
      std::cout << "Libraries:" << std::endl;
      for (auto &lib : config["libraries"]) {
        std::cout << "  " << std::string{lib["name"]} << std::endl;
        std::cout << "    " << std::string{lib["path"]} << std::endl;
      }
    }

    return 0;

  }
};

struct EnvCreateArgs : public argparse::Args {
  std::optional<std::string> &name = kwarg("name", "Name of the environment to create");
  int run() override{
    auto config = get_config();
    while (!name.has_value()) {
      std::string _name;
      std::cout << "Enter environment name: ";
      std::cin >> _name;
      if (config["environments"].contains(_name)) {
        std::cout << "Environment already exists" << std::endl;
        continue;
      }
      name.emplace(_name);
    }
    fs::path env_path = get_config_path().parent_path()/(*name);
    {
      fs::create_directories(env_path);
      json env = {
        {"name", *name}, 
        {"components", json::array()},
        {"nets", json::array()}
      };
      std::ofstream file(env_path/"env.json");
      file << env.dump(2);
      file.close();
    }
    {
      json cfg_env = {{"path", env_path/"env.json"}};
      config["environments"].push_back(cfg_env);
      save_config(config);
    }
    return 0;
  }
};

struct EnvArgs : public argparse::Args {
  EnvCreateArgs &create = subcommand("create");
  int run() override{
    return run_subcommands();
  }
};

struct LibsListArgs : public argparse::Args {
  int run() override
  {
    auto config = get_config();
    std::cout << "Libraries:" << std::endl;
    for (auto &lib : config["libraries"]) {
      std::cout << "  " << lib << std::endl;
    }
    return 0;
  }
};

struct LibsArgs : public argparse::Args {
  LibsListArgs &list = subcommand("list");
  int run() override{
    return run_subcommands();
  }
};

struct MyArgs : public argparse::Args {
  // std::string &binary = arg("flash_binary", "Path to binary file to load into flash");
  // std::optional<std::string> &uart0 = kwarg("uart0", "Path to UART0 file, can be PTY");
  // unsigned long long &max_ticks = kwarg("max_ticks", "Number of ticks to run").set_default(std::numeric_limits<unsigned long long>::max());
  // bool &core0_trace = kwarg("core0.trace", "Enable trace for core 0").set_default(false);
  // bool &core1_trace = kwarg("core1.trace", "Enable trace for core 1").set_default(false);
  // bool &core1_enable = kwarg("core1.enable", "Enable core 1").set_default(false);
  // bool &epd3in7 = flag("epd3in7", "Enable Waveshare EPD 3.7in display");

  ListArgs &list = subcommand("list");
  LibsArgs &libs = subcommand("libs");
  RunArgs &run = subcommand("run");
  EnvArgs &env = subcommand("envs");
};

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
  // args.print();

  return args.run_subcommands();


  // {
    
  //   auto &sim = Simulation::get();
  //   sim.vcd().top().add_item(g_rp2040.vcd());
  //   sim.vcd().open("/tmp/rp2040.vcd");
  //   g_rp2040.vcd().disable();
  //   sim.vcd().write_header();
    
  //   sim.run(args.max_ticks);
  // }
  // simulation_complete();
}