#pragma once

#include "ext/io/spidev.hpp"
// #include "display_driver.hpp"
#include <memory>

class DisplayDriver;

class DisplayInterface {
public:
  DisplayInterface(std::unique_ptr<DisplayDriver> &&display_drv);
  virtual ~DisplayInterface() = default;
  DisplayDriver &display_driver();
  void setBusy(bool busy) { m_busy_pin.set_drive_value(busy); }
protected:
  NetConnection &busy_pin() { return m_busy_pin; }
private:
  NetConnection m_busy_pin;
  std::unique_ptr<DisplayDriver> m_display_drv;
};

class SPIDisplay final : public SPIDev, public DisplayInterface{
public:
  SPIDisplay(std::unique_ptr<DisplayDriver> &&display_drv);
  virtual ~SPIDisplay() = default;

  NetConnection &dc_pin() { return m_dc_pin; }
  NetConnection &reset_pin() { return m_reset_pin; }

  void spi_cs_changed(bool cs) override;
  void spi_done(uint8_t *data, size_t len) override;

protected:
private:

  class Reset_Pin final : public NetConnection{
  public:
    Reset_Pin(SPIDisplay &dev) : m_dev{dev}{}
    void net_state_changed() override{
      bool val = connected_net()->digital_read();
      std::cerr << "Reset_Pin::net_state_changed reset=" << val << "\n";
    }
  protected:
  private:
    SPIDisplay &m_dev;
  } m_reset_pin;
  NetConnection m_dc_pin;

};