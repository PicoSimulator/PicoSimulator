#include "gui.hpp"
#include <GL/gl.h>


SFMLDisplay::Orientation SFMLDisplay::rotate(Orientation orientation, Orientation rotation)
{
  return Orientation((int(orientation) + int(rotation)) % 4);
}

SFMLDisplay::SFMLDisplay(unsigned width, unsigned height, Orientation orientation, unsigned scale)
    : thread{&SFMLDisplay::threadRun, this}
    , width{width}, height{height}, orientation{orientation}, scale{scale}
    , frozen{false}
    , running{true}
{
  std::cerr << "From SFMLDisplay\n";
  texture.create(width, height);
  thread.launch();
}
SFMLDisplay::~SFMLDisplay()
{
  // thread.terminate();
  running = false;
  thread.wait();
}

void SFMLDisplay::handle_events()
{
  sf::Event event;
  while (window.pollEvent(event))
  {
    if (event.type == sf::Event::Closed)
    {
      close();
    }
    if (event.type == sf::Event::KeyPressed)
    {
      switch (event.key.code)
      {
      case sf::Keyboard::R:
        setOrientation(rotate(orientation, event.key.shift
                                               ? SFMLDisplay::Orientation::ROTATE_90
                                               : SFMLDisplay::Orientation::ROTATE_270));

        break;
      case sf::Keyboard::Space:
        freeze(!frozen);
        render();
        break;
      case sf::Keyboard::Equal:
        if (event.key.control && scale < 5)
          setScale(scale + 1);
        break;
      case sf::Keyboard::Hyphen:
        if (event.key.control && scale > 1)
          setScale(scale - 1);
        break;
      case sf::Keyboard::Escape:
        close();
        break;
      }
    }
  }
}

void SFMLDisplay::threadRun()
{
  threadSetup();
  while (running)
  {
    threadLoop();
  }
  threadCleanup();
}

void SFMLDisplay::threadSetup()
{
  std::cerr << "From threadSetup\n";
  create_window();
  std::cerr << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
  std::cerr << "OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl;
  std::cerr << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
  window.setActive(true);
  window.clear(sf::Color::White);
  window.display();
  window.setActive(false);
}

void SFMLDisplay::threadLoop()
{


  
  window.setActive(true);

  if (!window.isOpen())
  {
    running = false;
    return;
  }

  handle_events();
  if (pending)
  {
    
    display_update();
    render();
    pending = false;
  }
  window.setActive(false);
}

void SFMLDisplay::threadCleanup()
{
  std::cerr << "From threadCleanup\n";
  window.close();
}

void SFMLDisplay::setOrientation(Orientation orientation)
{
  this->orientation = orientation;
  create_window();
  render(true);
}

void SFMLDisplay::setScale(unsigned scale)
{
  this->scale = scale;
  create_window();
  render(true);
}

unsigned SFMLDisplay::getWidth() const
{
  return width;
}

unsigned SFMLDisplay::getHeight() const
{
  return height;
}

void SFMLDisplay::freeze(bool frozen)
{
  this->frozen = frozen;
}

void SFMLDisplay::update_pending()
{
  pending = true; 
}

void SFMLDisplay::render(bool force)
{
  printf("render\n");
  if (!frozen)
  {
    sprite.setTexture(texture);
  }
  else if (!force)
  {
    return;
  }
  float rotations[] = {0, 270, 180, 90};
  sprite.setRotation(rotations[int(orientation)]);
  sprite.setScale(scale, scale);
  switch (orientation)
  {
  case Orientation::ROTATE_0:
    break;
  case Orientation::ROTATE_90:
    sprite.setPosition(0, window.getSize().y - 1);
    break;
  case Orientation::ROTATE_180:
    sprite.setPosition(window.getSize().x - 1, window.getSize().y - 1);
    break;
  case Orientation::ROTATE_270:
    sprite.setPosition(window.getSize().x - 1, 0);
    break;
  }
  window.draw(sprite);
  window.display();
}

void SFMLDisplay::create_window()
{
  sf::VideoMode mode = int(orientation) % 2 ? sf::VideoMode(height * scale, width * scale) : sf::VideoMode(width * scale, height * scale);
  window.create(mode, "SFMLDisplay Simulator", sf::Style::Close | sf::Style::Titlebar);
  window.setFramerateLimit(60);
}

void SFMLDisplay::close()
{
  window.close();
  exit(0);
}

SimpleEPaperDisplay::SimpleEPaperDisplay(DisplayDriver &driver, const std::map<uint32_t, sf::Color> &colormap, Orientation orientation)
    : SFMLDisplay{driver.getWidth(), driver.getHeight(), orientation}, driver{driver}, xOff{0}, yOff{0}, colors{colormap}
{
  driver.set_display_update_callback(std::bind(&SimpleEPaperDisplay::display_update, this));
  driver.set_event_callback(std::bind(&SFMLDisplay::handle_events, this));
}

SimpleEPaperDisplay::SimpleEPaperDisplay(DisplayDriver &driver, const std::map<uint32_t, sf::Color> &colormap, unsigned width, unsigned height, Orientation orientation)
    : SFMLDisplay{width, height, orientation}, driver{driver}, xOff{0}, yOff{0}, colors{colormap}
{
  driver.set_display_update_callback(std::bind(&SimpleEPaperDisplay::display_update, this));
  driver.set_event_callback(std::bind(&SFMLDisplay::handle_events, this));
}

DisplayDriver &SimpleEPaperDisplay::getDriver()
{
  return driver;
}

const std::map<uint32_t, sf::Color> SimpleEPaperDisplay::gray4 = {
    {0, sf::Color{0, 0, 0}},
    {1, sf::Color{85, 85, 85}},
    {2, sf::Color{170, 170, 170}},
    {3, sf::Color{255, 255, 255}},
};
const std::map<uint32_t, sf::Color> SimpleEPaperDisplay::bwr = {
    {0, sf::Color{0, 0, 0}},
    {1, sf::Color{255, 255, 255}},
    {2, sf::Color{255, 0, 0}},
    {3, sf::Color{0, 0, 0}},
};
const std::map<uint32_t, sf::Color> SimpleEPaperDisplay::bwy = {
    {0, sf::Color{0, 0, 0}},
    {1, sf::Color{255, 255, 255}},
    {2, sf::Color{0, 0, 255}},
    {3, sf::Color{0, 0, 0}},
};

void SimpleEPaperDisplay::display_update()
{
  sf::Image img;
  img.create(driver.getWidth(), driver.getHeight(), sf::Color::White);
  for (unsigned x = 0; x < driver.getWidth(); x++)
  {
    for (unsigned y = 0; y < driver.getHeight(); y++)
    {
      uint32_t color_idx = driver.getPixel(x, y);
      sf::Color color = colors.at(color_idx);
      img.setPixel(x, y, color);
    }
  }
  texture.loadFromImage(img);
  render();
}

SimulatedEPaperDisplay::SimulatedEPaperDisplay(EPaperDisplayDriver &driver, const std::vector<Pigment> &pigments, unsigned width, unsigned height, Orientation orientation)
    : SFMLDisplay{width, height, orientation}, driver{driver}, pigments{pigments}
{
  driver.set_display_update_callback(std::bind(&SimulatedEPaperDisplay::display_update, this));
  driver.set_frame_update_callback(std::bind(&SimulatedEPaperDisplay::frame_update, this, std::placeholders::_1));
  // driver.set_event_callback(std::bind(&SFMLDisplay::handle_events, this));
  pixels.resize(width);
  for (auto &row : pixels)
  {
    row.resize(height);
    for (auto &pixel : row)
    {
      pixel.resize(pigments.size());
      for (auto &pigment : pixel)
      {
        pigment = 0.5;
      }
    }
  }
}

const std::vector<SimulatedEPaperDisplay::Pigment> &SimulatedEPaperDisplay::bw()
{
  static const std::vector<Pigment> bw = {
      {0, 0, 0, 0.01},
      {1, 1, 1, -0.01},
  };
  return bw;
}

void SimulatedEPaperDisplay::display_update()
{
  handle_events();
  sf::Image img;
  img.create(driver.getWidth(), driver.getHeight(), sf::Color::White);
  SSD1677DisplayDriver &driver = static_cast<SSD1677DisplayDriver &>(this->driver);
  for (int i = 0; i < driver.max_frames(); i++) {
    //TODO fix frame speed
    driver.set_frame_num(i);
    frame_update(1);
    for (unsigned x = 0; x < getWidth(); x++)
    {
      for (unsigned y = 0; y < getHeight(); y++)
      {
        const auto &pigments = pixels[x][y];
        std::vector<int> indices;
        int minidx = 0;
        float maxmin = -1;
        float min = 1;
        unsigned j;
        for (unsigned i = 0; i < pigments.size(); i++)
        {
          minidx = 0;
          min = 2;
          for (j = 0; j < pigments.size(); j++)
          {
            float p = pigments[j];
            if (p <= maxmin)
            {
              continue;
            }
            if (p < min)
            {
              minidx = j;
            }
          }
          maxmin = pigments[minidx];
          indices.push_back(minidx);
        }
        sf::Color color{};
        uint8_t gray = 0;
        for (auto idx : indices) {
          gray += this->pigments[idx].r * 255 * pigments[idx];
        }
        color.r = gray;
        color.g = gray;
        color.b = gray;
  
        img.setPixel(x, y, color);
      }
    }
    texture.loadFromImage(img);
    render();
    handle_events();
  }
  clear_pending();
}

void SimulatedEPaperDisplay::frame_update(float dt)
{
  for (unsigned x = 0; x < getWidth(); x++)
  {
    for (unsigned y = 0; y < getHeight(); y++)
    {
      auto &pigments = pixels[x][y];
      auto pixel_voltage = driver.getPixelVoltage(x, y);
      for (unsigned i = 0; i < pigments.size(); i++)
      {
        pigments[i] += this->pigments[i].charge * dt * pixel_voltage;
        pigments[i] = std::clamp(pigments[i], 0.0f, 1.0f);
      }
    }
  }
  // display_update();
}