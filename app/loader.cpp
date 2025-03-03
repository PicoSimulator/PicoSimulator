#include "loader.hpp"
#include <dlfcn.h>

std::unique_ptr<IODevice> create_device(const std::string &name){
  std::unique_ptr<IODevice> (*create_device)(const std::string &name);

  std::string libname = "lib" + name.substr(0, name.find(':')) + ".so";
  void *handle = dlopen(libname.c_str(), RTLD_LAZY);
  if (!handle) {
    std::cerr << "Failed to open libepdsim.so: " << dlerror() << std::endl;
    return nullptr;
  }
  create_device = (std::unique_ptr<IODevice> (*)(const std::string &name))dlsym(handle, "create_device");
  if (!create_device) {
    std::cerr << "Failed to find create_device in libepdsim.so: " << dlerror() << std::endl;
    return nullptr;
  }
  auto dev = create_device(name.substr(name.find(':')+1));
  printf("dev: %p\n", dev.get());
  return dev;
}

