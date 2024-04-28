#pragma once

// Digital Pin
class Pin {
public:
  operator bool() const { return read(); }
  bool read() const { return m_value; }
  void write(bool value) { m_value = value; /* check net */ }
protected:
private:
  enum class Mode {
    Input,
    Output,
    InputPull,
  };
  bool m_value;
};