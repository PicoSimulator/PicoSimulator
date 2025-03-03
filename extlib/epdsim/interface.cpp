#include "interface.hpp"
#include "display_driver.hpp"

DisplayInterface::DisplayInterface(std::unique_ptr<DisplayDriver> &&display_drv)
: m_display_drv{std::move(display_drv)}
{
  m_display_drv->set_interface(*this);
}

DisplayDriver &DisplayInterface::display_driver() { return *m_display_drv; }


SPIDisplay::SPIDisplay(std::unique_ptr<DisplayDriver> &&display_drv)
: SPIDev()
, DisplayInterface(std::move(display_drv))
, m_reset_pin{*this}
{
  display_driver().set_interface(*this);
  add_named_pin("D/~C", m_dc_pin);
  add_named_pin("~RESET", m_reset_pin);
  add_named_pin("BUSY", busy_pin());
  busy_pin().set_drive_value(false);
  busy_pin().set_drive_strength(255);
}

void SPIDisplay::spi_cs_changed(bool cs)
{
  if (cs)
    return;
  // std::cerr << "spi_cs_changed: " << cs << std::endl;
  spi_start(nullptr, 1);
}

void SPIDisplay::spi_done(uint8_t *data, size_t len)
{
  // std::cerr << "spi_done: " << len << std::endl;
  auto fn = m_dc_pin.connected_net()->digital_read() ? &DisplayDriver::writeData : &DisplayDriver::writeCommand;
  for (size_t i = 0; i < len; i++)
    (display_driver().*fn)(data[i]);
  spi_start(nullptr, 1);
}