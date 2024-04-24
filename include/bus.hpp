#pragma once

#include "async.hpp"
#include <coroutine>
#include <vector>
#include <algorithm>
#include <iostream>

class BusMaster{
public:
  struct promise_type;
  using Handle = std::coroutine_handle<BusMaster::promise_type>;
  struct promise_type {
    BusMaster get_return_object() { return BusMaster{Handle::from_promise(*this)}; }
    std::suspend_always initial_suspend() { return {}; }
    std::suspend_always final_suspend() noexcept { return {}; }
    void return_void() {}
    void unhandled_exception() {}
    enum SuspendReason {
      SuspendReasonNone,
      SuspendReasonWaitForTick,
      SuspendReasonWaitForBus,
    };
    SuspendReason m_suspend_reason;
  };
  Handle m_handle;
  BusMaster(Handle handle) : m_handle(handle){}
  void resume(){
    m_handle.resume();
  }
};

class Bus{
public:
  void registerBusMaster(BusMaster *master){
    m_masters.push_back(master);
  }
  void deregisterBusMaster(BusMaster *master){
    m_masters.erase(std::find(m_masters.begin(), m_masters.end(), master));
  }
protected:
private:
  std::vector<BusMaster*> m_masters;
};

namespace BusRead{
  template<class Addr>
  struct byte {
    IReadPort<Addr> &port;
    Addr addr;
    auto operator co_await(){
      struct awaiter{
        IReadPort<Addr> &port;
        Addr addr;
        uint8_t data;
        awaiter(IReadPort<Addr> &port, Addr addr) : port(port), addr(addr){}
        bool await_ready(){
          return false;
        }
        void await_suspend(std::coroutine_handle<> handle){
          port.read_byte(addr, data);
          handle.resume();
        }
        auto await_resume(){
          return data;
        }
      };
      return awaiter{port, addr};
    }
  };
  template<class Addr>
  struct halfword {
    IReadPort<Addr> &port;
    Addr addr;
    auto operator co_await(){
      struct awaiter{
        IReadPort<Addr> &port;
        Addr addr;
        uint16_t data;
        awaiter(IReadPort<Addr> &port, Addr addr) : port(port), addr(addr){}
        bool await_ready(){
          return true;
        }
        void await_suspend(std::coroutine_handle<> handle){
          std::cout << "await suspend read_halfword" << std::endl;
          // handle.resume();
        }
        auto await_resume(){
          std::cout << "await resume read_halfword" << std::endl;
          port.read_halfword(addr, data);
          return data;
        }
      };
      return awaiter{port, addr};
    }
  };
  template<class Addr>
  struct word {
    IReadPort<Addr> &port;
    Addr addr;
    auto operator co_await(){
      struct awaiter{
        IReadPort<Addr> &port;
        Addr addr;
        uint32_t data;
        awaiter(IReadPort<Addr> &port, Addr addr) : port(port), addr(addr){}
        bool await_ready(){
          return false;
        }
        void await_suspend(std::coroutine_handle<> handle){
          port.read_word(addr, data);
          handle.resume();
        }
        auto await_resume(){
          return data;
        }
      };
      return awaiter{port, addr};
    }
  };
};

template<class Data, class Addr>
struct BusWrite{
  IWritePort<Addr> &port;
  Addr addr;
  Data data;
  auto operator co_await(){
    struct awaiter{
      IWritePort<Addr> &port;
      Addr addr;
      Data data;
      awaiter(IWritePort<Addr> &port, Addr addr, Data data) : port(port), addr(addr), data(data){}
      bool await_ready(){
        return false;
      }
      void await_suspend(std::coroutine_handle<> handle){
        port.write_word(addr, data);
        handle.resume();
      }
      void await_resume(){}
    };
    return awaiter{port, addr, data};
  }
};

enum SuspendReason {
  SuspendReasonNone,
  SuspendReasonWait,
  SuspendReasonFault,
  SuspendReasonStall,
};


// auto bus_master_suspend(SuspendReason reason)
// {
//   struct awaitable
//   {
//     bool await_ready() const noexcept { return false; }
//     void await_suspend(std::coroutine_handle<> handle) noexcept
//     {
//       // suspend the bus master
//     }
//     void await_resume() const noexcept {}
//   };
//   return awaitable{};
// }