#include "display_driver.hpp"

#include <cstring>

#define TEST_X 129
#define TEST_Y 169

void DisplayDriver::set_display_update_callback(std::function<void()> callback)
{
  display_update_callback = callback;
}

void DisplayDriver::set_event_callback(std::function<void()> callback)
{
  event_callback = callback;
}

SFMLDisplay *DisplayDriver::display() const
{
  return m_display.get();
}

void DisplayDriver::set_interface(DisplayInterface &interface)
{
  m_interface = &interface;
}

DisplayInterface &DisplayDriver::interface() const
{
  return *m_interface;
}

void DisplayDriver::display_update()
{
  if (display_update_callback)
  {
    display_update_callback();
  }
}

void DisplayDriver::event()
{
  if (event_callback)
  {
    event_callback();
  }
}

void EPaperDisplayDriver::set_frame_update_callback(std::function<void(float)> callback)
{
  frame_update_callback = callback;
}

float EPaperDisplayDriver::getPixelVoltage(uint32_t x, uint32_t y)
{
  uint32_t pixel = getPixel(x, y);
  float voltage = waveforms[pixel][frame_num];
  return voltage;
}

EPaperDisplayDriver::EPaperDisplayDriver(unsigned int num_waveforms)
    : waveforms{num_waveforms}, frame_num{0} {}

void EPaperDisplayDriver::display_update()
{
  // called from main simulation thread so is safe!
  interface().setBusy(true);
  // trigger display update on render thread.
  display()->update_pending();
  std::cerr << "EPaperDisplayDriver::display_update()" << std::endl;
}

void EPaperDisplayDriver::frame_update(float dt)
{
  if (frame_update_callback)
  {
    frame_update_callback(dt);
  }
}

void EPaperDisplayDriver::resetWaveforms()
{
  for (auto &waveform : waveforms)
  {
    waveform.clear();
  }
}

std::vector<float> &EPaperDisplayDriver::getWaveform(uint32_t waveform)
{
  return waveforms[waveform];
}

SSD1677DisplayDriver::SSD1677DisplayDriver()
    : EPaperDisplayDriver{5}
    , BWRam{Ram[0]}
    , RedRam{Ram[1]}
{
  reset();
}

void SSD1677DisplayDriver::writeCommand(uint8_t command)
{
  // std::cerr << "SSD1677DisplayDriver::writeCommand(" << (int)command << ")" << std::endl;
  event();
  switch (current_command)
  {
  case 0x04:
    regen_voltage_lut();
    break;
  case 0x12:
    reset();
    break;
  case 0x32:
    recalc_waveforms();
    break;
  }
  current_command = command;
  offset = 0;
  switch (command)
  {
  case 0x20:
    display_update();
    if (displayMode == 2) {
      memcpy(RedRam, BWRam, (960/8)*680);
      // std::swap(BWRam, RedRam);
    }
    break;
  }
}

void write_u10(unsigned char data, uint16_t &val, unsigned offset)
{
  // write 10 bits
  if (offset == 0)
  {
    val = (val & 0x300) | (data & 0x0FF);
  }
  else
  {
    val = (val & 0x0FF) | ((data & 0x03) << 8);
  }
}

template <unsigned long N>
void write_n_u10(unsigned char data, std::array<uint16_t *, N> vals, unsigned offset)
{
  unsigned idx = offset / 2;
  if (idx >= N)
    return;
  write_u10(data, *vals[idx], offset % 2);
}

float calc_vsh(uint8_t data)
{
  if (data >= 0x8e && data <= 0xce)
  {
    return 2.4 + 0.1 * (data - 0x8e);
  }
  else if (data >= 0x23 && data <= 0x4b)
  {
    return 9 + 0.2 * (data - 0x23);
  }
  return 0;
}

float calc_vsl(uint8_t data)
{
  if (data >= 0x1a && data <= 0x3a && data % 2 == 0)
  {
    return -9 - 0.25 * (data - 0x1a);
  }
  return 0;
}

void SSD1677DisplayDriver::writeData(uint8_t data)
{
  // std::cerr << "SSD1677DisplayDriver::writeData(" << (int)data << ")" << std::endl;
  event();
  switch (current_command)
  {
  case 0x04: // set source driving voltage
    if (offset == 0)
      VSH1 = calc_vsh(data);
    else if (offset == 1)
      VSH2 = calc_vsh(data);
    else if (offset == 2)
      VSL = calc_vsl(data);
    offset++;
  case 0x11: // data entry sequence (xy inc/dec, address mode)
    data & 1 ? xInc = 1 : xInc = -1;
    data & 2 ? yInc = 1 : yInc = -1;
    data & 4 ? addrMode = true : addrMode = false;
    break;
  case 0x21: // RAM masking
    ramMask = ~((data >> 2) & 1 | (data >> 5) & 2);
    ramInv = ((data >> 3) & 1 | (data >> 6) & 2);
    break;
  case 0x24:
    ram_write_byte(BWRam, data);
    break;
  case 0x26:
    ram_write_byte(RedRam, data);
    break;
  case 0x32:
    if (offset < 112)
      LUT[offset++] = data;
    break;
  case 0x37:
    if (offset++ == 1) {
      displayMode = data? 2 : 1;
    }
    break;
  case 0x44:
    write_n_u10(data, std::array<uint16_t *, 2>{&xStart, &xEnd}, offset++);
    break;
  case 0x45:
    write_n_u10(data, std::array<uint16_t *, 2>{&yStart, &yEnd}, offset++);
    break;
  case 0x4e:
    write_n_u10(data, std::array<uint16_t *, 1>{&xAddr}, offset++);
    break;
  case 0x4f:
    write_n_u10(data, std::array<uint16_t *, 1>{&yAddr}, offset++);
    break;
  }
}

uint8_t SSD1677DisplayDriver::readData() {}

void SSD1677DisplayDriver::reset()
{
  xStart = 0;
  xEnd = 0;
  yStart = 0;
  yEnd = 0;
  xAddr = 0;
  yAddr = 0;
  offset = 0;

  ramMask = 3;
  ramInv = 0;

  xInc = 1;
  yInc = 1;
  addrMode = false;
  for (int i = 0; i < (960 / 8) * 680; i++)
  {
    BWRam[i] = 0;
    RedRam[i] = 0;
  }
  for (int i = 0; i < 112; i++)
  {
    LUT[i] = 0;
  }
}

uint32_t SSD1677DisplayDriver::getPixel(uint32_t x, uint32_t y)
{
  bool bw_bit = ram_bit(BWRam, x, y);
  bool red_bit = ram_bit(RedRam, x, y);
  uint8_t pixel = (red_bit << 1) | bw_bit;
  return (pixel ^ ramInv) & ramMask;
}

uint32_t SSD1677DisplayDriver::getWidth() const
{
  return 960;
}

uint32_t SSD1677DisplayDriver::getHeight() const
{
  return 680;
}

// void SSD1677DisplayDriver::create_display(unsigned width, unsigned height)
// {
//   std::unique_ptr<SFMLDisplay> display = std::make_unique<SimulatedEPaperDisplay>(*this, SimulatedEPaperDisplay::bw(), width, height, SFMLDisplay::Orientation::ROTATE_90);
//   setDisplay(std::move(display));
// }

void SSD1677DisplayDriver::ram_write_byte(uint8_t *ram, uint8_t data)
{
  for (int i = 0; i < 8; i++)
  {
    // this can be optimised to only two writes
    int bit_idx = (xAddr % 8);
    int ram_idx = (xAddr) / 8 + yAddr * 960 / 8;
    ram[ram_idx] = (ram[ram_idx] & ~(1 << bit_idx)) | (((data >> (7 - i)) & 1) << bit_idx);

    // handle different addressing modes
    // and x/y increment/decrement

    if (!addrMode)
    {
      if (xAddr == xEnd)
      {
        xAddr = xStart;
        if (yAddr == yEnd)
        {
          yAddr = yStart;
        }
        else
        {
          yAddr += yInc;
        }
        break;
      }
      else
      {
        xAddr += xInc;
      }
    }
    else
    {
      if (yAddr == yEnd)
      {
        yAddr = yStart;
        if (xAddr == xEnd)
        {
          xAddr = xStart;
        }
        else
        {
          xAddr += xInc;
        }
        break;
      }
      else
      {
        yAddr += yInc;
      }
    }
  }
}

bool SSD1677DisplayDriver::ram_bit(const uint8_t *ram, uint32_t x, uint32_t y)
{
  int bit_idx = (x % 8);
  int ram_idx = x / 8 + y * 960 / 8;
  return (ram[ram_idx] >> bit_idx) & 1;
}

void SSD1677DisplayDriver::recalc_waveforms()
{
  printf("recalc waveforms\n");
  for (int i = 0; i < 4; i++)
  {
    auto &waveform = getWaveform(i);
    waveform.clear();
    for (int grp = 0; grp < 10; grp++)
    {
      int repeat = LUT[54 + 5 * grp] + 1;
      for (int _ = 0; _ < repeat; _++)
      {
        for (int frame = 0; frame < 4; frame++)
        {
          uint8_t grp_val = LUT[10 * i + grp];
          uint8_t frame_val = (grp_val >> (6 - 2 * frame)) & 3;
          uint8_t frame_cnt = LUT[50 + 5 * grp + frame];
          for (int _2 = 0; _2 < frame_cnt; _2++)
            waveform.push_back(voltage_lut[frame_val]);
        }
      }
    }
  }
  auto &waveform = getWaveform(3);
}

void SSD1677DisplayDriver::regen_voltage_lut()
{
  voltage_lut[0] = 0.0;
  voltage_lut[1] = VSH1;
  voltage_lut[2] = VSL;
  voltage_lut[3] = VSH2;
  printf("VSH1: %f, VSL: %f, VSH2: %f\n", VSH1, VSL, VSH2);
  recalc_waveforms();
}
