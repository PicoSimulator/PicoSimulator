#pragma once

#include <string>

class Component{
public:
  virtual bool set_param(const std::string &name, const std::string &value) { return false; }
  virtual void ready() {}
protected:
private:
};