#pragma once

#include <coroutine>
#include <utility>
#include <iostream>

// co_return void
class Task;

// co_yield some data
template<class T>
class Generator;

// can be awaited and returns a value
template<class T>
class Awaitable;

class Task {
public:
  std::coroutine_handle<> get_handle() const { return m_handle; }
// protected:
// private:
    struct promise_type;
    using Handle = std::coroutine_handle<promise_type>;
    Task (Handle handle) : m_handle(handle) {}
    struct promise_type {
        Task get_return_object() { return Task{Handle::from_promise(*this)}; }
        std::suspend_never initial_suspend() noexcept { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }
        void return_void() {}
        void unhandled_exception() { std::rethrow_exception(std::current_exception()); }
    };
    Handle m_handle;
};

// template<class T>
// class Generator {
// public:
//     explicit operator bool() const { 
//         fill();
//         return !m_handle.done(); 
//     }
//     T operator()() {
//         fill();
//         full = false;
//         return std::move(m_handle.promise().m_value);
//     }
// protected:
// private:
//     struct Promise;
//     using Handle = std::coroutine_handle<Promise>;
//     using promise_type = Promise;
//     struct Promise {
//         T m_value;
//         Generator get_return_object() const { return Generator{Handle::from_promise(*this)}; }
//         std::suspend_always initial_suspend() noexcept { return {}; }
//         std::suspend_always final_suspend() noexcept { return {}; }
//         void return_void() {}
//         void unhandled_exception() { std::terminate(); }
//         template<std::convertible_to<T> From>
//         std::suspend_always yield_value(From&& value) {
//             m_value = std::forward<From>(value);
//             return {};
//         }
//     };

//     Handle m_handle;
//     bool full = false;
//     Generator(Handle handle) : m_handle(handle) {}
//     ~Generator() { m_handle.destroy(); }

//     void fill() {
//         if (!full) {
//             m_handle();
//             if (m_handle.promise().exception_)
//                 std::rethrow_exception(m_handle.promise().exception_);
//             full = true;
//         }
//     }


// };

template<class T>
class Awaitable {
public:
    bool await_ready() const noexcept { return m_handle.done(); }
    void await_suspend(std::coroutine_handle<> coro) const noexcept { 
      m_handle.promise().precursor = coro;
    }
    T await_resume() { return std::move(m_handle.promise().m_value); }
    struct promise_type;
    using Handle = std::coroutine_handle<promise_type>;
    struct promise_type {
        Awaitable get_return_object() { return Awaitable{Handle::from_promise(*this)}; }
        std::suspend_never initial_suspend() noexcept { return {}; }
        auto final_suspend() noexcept {
          struct Awaiter {
            bool await_ready() const noexcept { return false; }
            std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) const noexcept {
              auto precursor = h.promise().precursor;
              if (precursor)
                return precursor;
              return std::noop_coroutine();
            }
            void await_resume() const noexcept {}
          };
          return Awaiter{};
        }
        void return_value(T value) requires (!std::is_void_v<T>) { 
          m_value = std::move(value); 
          }
        void unhandled_exception() {  std::rethrow_exception(std::current_exception()); }
        T m_value;
        std::coroutine_handle<> precursor;
    };
protected:
private:
    Awaitable(Handle handle) : m_handle(handle) {}
    Handle m_handle;
};

template<>
class Awaitable<void> {
public:
    bool await_ready() const noexcept { return m_handle.done(); }
    void await_suspend(std::coroutine_handle<> coro) const noexcept { 
      m_handle.promise().precursor = coro;
    }
    void await_resume() {
      if (m_handle.promise().m_exception)
        std::rethrow_exception(m_handle.promise().m_exception);
    }
    struct promise_type;
    using Handle = std::coroutine_handle<promise_type>;
    struct promise_type {
        Awaitable get_return_object() { return Awaitable{Handle::from_promise(*this)}; }
        // start by running our task and seeing what happens
        std::suspend_never initial_suspend() noexcept { return {}; }
        auto final_suspend() noexcept {
          struct Awaiter {
            bool await_ready() const noexcept { return false; }
            std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) const noexcept {
              auto precursor = h.promise().precursor;
              if (precursor)
                return precursor;
              return std::noop_coroutine();
            }
            void await_resume() const noexcept {}
          };
          return Awaiter{};
        }
        void return_void() {}
        void unhandled_exception() { m_exception = std::current_exception(); }
        std::coroutine_handle<> precursor;
        std::exception_ptr m_exception = nullptr;
    };
protected:
private:
    Awaitable(Handle handle) : m_handle(handle) {}
    Handle m_handle;
};


// template<class T>
// class Awaitable : std::coroutine_handle<Awaitable::promise_type>{
// public:
// // protected:
// // private:
//   struct promise_type{
//     Awaitable get_return_object() { return {std::coroutine_handle<promise_type>::from_promise(*this)}; }
//     std::suspend_never initial_suspend() noexcept { return {}; }
//     std::suspend_never final_suspend() noexcept { return {}; }
//     void unhandled_exception();
//     void return_void() requires(std::is_void_v<T>) {}
//     T return_value(T value) requires(!std::is_void_v<T>) {}
//   };
// };

// template<class T>
// struct Awaitable
// {
//     struct promise_type
//     {
//         Awaitable get_return_object() { return {}; }
//         std::suspend_never initial_suspend() { return {}; }
//         std::suspend_never final_suspend() noexcept { return {}; }
//         void return_void() {}
//         void unhandled_exception() {}
//     };
// };