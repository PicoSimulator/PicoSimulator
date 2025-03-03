#include "factory.hpp"
#include "display_driver.hpp"
#include "interface.hpp"

std::unique_ptr<IODevice> create_device(const std::string &name){
  std::unique_ptr<SPIDisplay> dev = nullptr;
  if (name == "epd3in7"){
    dev = std::make_unique<SPIDisplay>(std::make_unique<SSD1677DisplayDriver>());
    printf("dev: %p\n", dev.get());
    dev->display_driver().create_display(280, 480);
  }
  auto dev2 = std::unique_ptr<IODevice>(dev.release());
  printf("dev2 %p\n", dev2.get());
  return dev2;
}

const std::span<const std::string> device_names(){
  static const std::string names[] = {
    "epd3in7",
  };
  return std::span<const std::string>(names);
}