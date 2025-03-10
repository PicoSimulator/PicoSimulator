#include "factory.hpp"
#include "display_driver.hpp"
#include "interface.hpp"

std::unique_ptr<IODevice> create_device(const std::string &name){
  std::unique_ptr<SPIDisplay> dev = nullptr;
  if (name == "epd3in7"){
    auto drv = std::make_unique<SSD1677DisplayDriver>();
    auto display = std::make_unique<SimulatedEPaperDisplay>(
      *drv, 
      SimulatedEPaperDisplay::bw(),
      280, 480, 
      SFMLDisplay::Orientation::ROTATE_90);
      drv->set_display(std::move(display));
      dev = std::make_unique<SPIDisplay>(std::move(drv));
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