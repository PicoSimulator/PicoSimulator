#pragma once


#include <cstdint>
#include <SFML/Graphics.hpp>
#include <functional>
#include <map>
#include <memory>
#include <atomic>
#include "display_driver.hpp"

class DisplayDriver;
class EPaperDisplayDriver;
class SSD1677DisplayDriver;

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
    void threadRun();
    void threadSetup();
    void threadLoop();
    void threadCleanup();
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
  