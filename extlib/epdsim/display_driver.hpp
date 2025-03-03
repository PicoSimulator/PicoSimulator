#pragma once

#include <cstdint>
#include <SFML/Graphics.hpp>
#include <functional>
#include <map>
#include <memory>
#include <atomic>
#include "interface.hpp"

class SFMLDisplay;

class DisplayDriver {
public:
  DisplayDriver() = default;
  virtual void writeCommand(uint8_t command) = 0;
  virtual void writeData(uint8_t data) = 0;
  virtual uint8_t readData() = 0;
  virtual void reset() = 0;
  virtual uint32_t getPixel(uint32_t x, uint32_t y) = 0;
  virtual uint32_t getWidth() const = 0;
  virtual uint32_t getHeight() const = 0;
  void set_display_update_callback(std::function<void()> callback);
  void set_event_callback(std::function<void()> callback);
  virtual void create_display(unsigned width, unsigned height) = 0;
  SFMLDisplay *display() const;
  void set_interface(DisplayInterface &interface);
  DisplayInterface &interface() const;

protected:
  void display_update();
  void event();
  void setDisplay(std::unique_ptr<SFMLDisplay> &&display) { m_display = std::move(display); }
private:
  std::function<void()> display_update_callback;
  std::function<void()> event_callback;
  std::unique_ptr<SFMLDisplay> m_display;
  DisplayInterface *m_interface;
};

class EPaperDisplayDriver : public DisplayDriver {
public:
  void set_frame_update_callback(std::function<void(float)> callback);
  float getPixelVoltage(uint32_t x, uint32_t y);
  size_t max_frames() const { return waveforms[0].size(); }
  void set_frame_num(unsigned frame_num) { this->frame_num = frame_num; }
protected:
  EPaperDisplayDriver(unsigned int num_waveforms);
  void display_update();
  void frame_update(float dt);
  void resetWaveforms();
  void setBusy(bool busy);
  std::vector<float> &getWaveform(uint32_t waveform);
private:
  std::function<void(float)> frame_update_callback;
  std::vector<std::vector<float>> waveforms;
  unsigned frame_num;
};

class SSD1677DisplayDriver final : public EPaperDisplayDriver {
public:
  SSD1677DisplayDriver();
  void writeCommand(uint8_t command) override;
  void writeData(uint8_t data) override;
  uint8_t readData() override;
  void reset() override;
  uint32_t getPixel(uint32_t x, uint32_t y) override;
  uint32_t getWidth() const override;
  uint32_t getHeight() const override;
  void create_display(unsigned width, unsigned height) override;
private:
  void ram_write_byte(uint8_t *ram, uint8_t data);
  bool ram_bit(const uint8_t *ram, uint32_t x, uint32_t y);
  void recalc_waveforms();
  void regen_voltage_lut();

  // 0x03
  // VGH/VGL
  // 0x04
  float VSL, VSH1, VSH2;
  uint8_t current_command;
  // 0x20
  // void display_update();
  // 0x24
  uint8_t *BWRam;
  // 0x26
  uint8_t *RedRam;
  uint8_t Ram[2][(960/8)*680];
  // 0x32
  uint8_t LUT[112];
  // 0x37
  uint8_t displayMode;
  // 0x44
  uint16_t xStart, xEnd;
  // 0x45
  uint16_t yStart, yEnd;
  //0x4E
  uint16_t xAddr;
  // 0x4F
  uint16_t yAddr;

  uint16_t offset;
  uint8_t ramMask;
  uint8_t ramInv;
  int xInc, yInc;
  bool addrMode;

  float voltage_lut[4];
};

class SFMLDisplay {
public:
  enum class Orientation{
    ROTATE_0,
    ROTATE_90,
    ROTATE_180,
    ROTATE_270
  };
  static Orientation rotate(Orientation orientation, Orientation rotation);

  SFMLDisplay(unsigned width, unsigned height, Orientation orientation = Orientation::ROTATE_0, unsigned scale=1);
  ~SFMLDisplay();
  void setOrientation(Orientation orientation);
  void setScale(unsigned scale);
  unsigned getWidth() const;
  unsigned getHeight() const;
  // handles any events on the window
  void handle_events();
  // freezes the display, preventing it from updating
  // can be overridden if display resizes or rescales.
  void freeze(bool frozen);
  // set pending update to display.
  void update_pending();
  protected:
  // re-renders textures to the display.
  void render(bool force=false);
  // handles event loop and rendering
  void threadLoop();
  sf::Texture texture;
  virtual void frame_update(float dt) = 0;
  virtual void display_update() = 0;
  void clear_pending() { pending = false; }
private:
  void create_window();
  void close();
  sf::RenderWindow window;
  sf::Thread thread;
  unsigned width, height;
  Orientation orientation;
  unsigned scale;
  bool frozen;
  sf::Sprite sprite;
  std::atomic<bool> pending;
  bool running;
};

class SimpleEPaperDisplay final : public SFMLDisplay {
public:
  SimpleEPaperDisplay(DisplayDriver &driver, const std::map<uint32_t, sf::Color> &colormap, Orientation orientation = Orientation::ROTATE_0);
  SimpleEPaperDisplay(DisplayDriver &driver, const std::map<uint32_t, sf::Color> &colormap, unsigned width, unsigned height, Orientation orientation = Orientation::ROTATE_0);
  DisplayDriver &getDriver();
  static const std::map<uint32_t, sf::Color> gray4;
  static const std::map<uint32_t, sf::Color> bwr;
  static const std::map<uint32_t, sf::Color> bwy;
private:
  void display_update();
  DisplayDriver &driver;
  unsigned xOff, yOff;
  const std::map<uint32_t, sf::Color> &colors;
};

class SimulatedEPaperDisplay final : public SFMLDisplay {
public:
  struct Pigment{
    float r,g,b;
    float charge;
  };
  template<typename T>
  using vecx2 = std::vector<std::vector<T>>;
  SimulatedEPaperDisplay(EPaperDisplayDriver &driver, const std::vector<Pigment> &pigments, unsigned width, unsigned height, Orientation orientation = Orientation::ROTATE_0);
  static const std::vector<Pigment> &bw();
private:
  // updates display textures based on pixel pigments
  void display_update();
  // updates the pigments based on elapsed time and waveform voltage
  void frame_update(float dt);
  EPaperDisplayDriver &driver;
  vecx2<std::vector<float>> pixels;
  const std::vector<Pigment> &pigments;
};