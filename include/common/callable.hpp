#pragma once

template<typename Ret, typename ...Args>
class ICallable<Ret(...Args)> {
public:
  virtual Ret operator()(Args ...args) = 0;
};