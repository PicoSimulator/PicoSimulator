#pragma once

class GPIO;

class GPIOSignal{
  friend class GPIO;
public:
  GPIOSignal(::GPIO *input_gpio = nullptr)
  : m_input_gpio{input_gpio}
  {}
  bool get_output() const { return m_output_value; }
  bool get_oe() const { return m_output_enable; }
  void set_output(bool value);
  void set_oe(bool oe);
  virtual void irq() {}
  virtual void input_changed() {}
  bool get_input() const;
  static GPIOSignal &dummy();
  ::GPIO *&gpio() { return m_gpio; }
protected:
private:
  bool m_output_enable = 0;
  bool m_output_value = 0;
  ::GPIO *m_gpio;
  const ::GPIO *m_input_gpio;
};

class GPIO{
public:
  virtual bool get_input() const = 0;
  virtual bool get_output() const = 0;
  virtual bool get_output_enable() const = 0;
  virtual void update_from_internal() = 0;
};