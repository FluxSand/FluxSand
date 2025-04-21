
#pragma once
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>

class Gpio {
 public:
  using Callback = std::function<void()>;

  Gpio(const std::string& chip_name, unsigned int line_num, bool is_output, int default_value = 1)
      : is_output_(is_output), value_(default_value), line_num_(line_num) {}

  void Write(int value) {
    if (!is_output_) throw std::runtime_error("Trying to write to input pin!");
    value_ = value;
  }

  int Read() const {
    return value_;
  }

  void EnableInterruptRisingEdgeWithCallback(Callback cb) {
    callback_ = std::move(cb);
    interrupt_enabled_ = true;
    interrupt_thread_ = std::thread([this]() {
      int last = value_;
      while (interrupt_enabled_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (value_ == 1 && last == 0 && callback_) {
          callback_();
        }
        last = value_;
      }
    });
  }

  ~Gpio() {
    interrupt_enabled_ = false;
    if (interrupt_thread_.joinable()) interrupt_thread_.join();
  }

 private:
  bool is_output_;
  int value_;
  unsigned int line_num_;
  Callback callback_;
  std::atomic<bool> interrupt_enabled_{false};
  std::thread interrupt_thread_;
};
