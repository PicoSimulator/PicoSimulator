#include "net.hpp"

class PullUp : public NetConnection {
public:
  PullUp() 
  : NetConnection{} 
  {
    set_drive_strength(1);
    set_drive_value(1);
  }
};

class PullDown : public NetConnection {
public:
  PullDown() 
  : NetConnection{} 
  {
    set_drive_strength(1);
    set_drive_value(0);
  }
};