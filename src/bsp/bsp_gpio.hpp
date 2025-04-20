#pragma once

#include <gpiod.h>

#include <atomic>
#include <cassert>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

/**
 * Gpio class
 *
 * Handles GPIO initialization, configuration (input/output),
 * read/write operations, and interrupt callback support.
 */
class Gpio {
 public:
  using Callback = std::function<void()>;

  /**
   * Constructor
   *
   * @param chip_name     GPIO chip name, e.g., "gpiochip0"
   * @param line_num      GPIO line number
   * @param is_output     true for output mode, false for input mode
   * @param default_value Initial value for output mode (default: 1)
   */
  Gpio(const std::string& chip_name, unsigned int line_num, bool is_output,
       int default_value = 1)
      : chip_(gpiod_chip_open_by_name(chip_name.c_str())),
        is_output_(is_output),
        line_num_(line_num) {
    assert(!chip_name.empty()); /* Ensure chip name is valid */

    if (!chip_) {
      std::perror("Failed to open GPIO chip");
    }

    line_ = gpiod_chip_get_line(chip_, line_num_);
    if (!line_) {
      gpiod_chip_close(chip_);
      std::perror("Failed to get GPIO line");
    }

    int ret = is_output_
                  ? gpiod_line_request_output(line_, nullptr, default_value)
                  : gpiod_line_request_input(line_, nullptr);
    if (ret < 0) {
      std::perror("Failed to configure GPIO line");
    }
  }

  /**
   * Destructor
   *
   * Releases allocated resources and stops interrupt thread.
   */
  ~Gpio() {
    running_ = false;

    if (interrupt_thread_.joinable()) {
      interrupt_thread_.join();
    }

    if (line_) {
      gpiod_line_release(line_);
    }

    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

  /**
   * Write a value to the GPIO line (only valid in output mode).
   *
   * @param value Value to write (0 or 1)
   */
  void Write(int value) {
    assert(is_output_);
    if (gpiod_line_set_value(line_, value) < 0) {
      std::perror("GPIO write failed");
    }
  }

  /**
   * Read the current GPIO value.
   *
   * @return Current GPIO value (0 or 1)
   */
  int Read() const {
    int value = gpiod_line_get_value(line_);
    if (value < 0) {
      std::perror("GPIO read failed");
    }
    return value;
  }

  /**
   * Enable rising edge interrupt and register callback.
   *
   * This starts a background thread to monitor GPIO events.
   *
   * @param cb Callback function to be invoked on rising edge.
   */
  void EnableInterruptRisingEdgeWithCallback(Callback cb) {
    if (is_output_) {
      std::perror("Cannot register interrupt on output GPIO");
    }

    /* If line is already in use, release it */
    if (line_) {
      gpiod_line_release(line_);
    }

    /* Request GPIO line for interrupt events */
    if (gpiod_line_request_rising_edge_events(line_, nullptr) < 0) {
      std::perror("Failed to enable rising edge interrupt");
    }

    callback_ = std::move(cb);
    running_ = true;

    interrupt_thread_ = std::thread(&Gpio::InterruptWaitLoop, this);
  }

 private:
  /**
   * Background thread loop that waits for GPIO rising edge events.
   */
  void InterruptWaitLoop() {
    while (running_) {
      int ret = gpiod_line_event_wait(line_, nullptr);  // block until event
      if (ret == 1 && callback_) {
        struct gpiod_line_event event;
        if (gpiod_line_event_read(line_, &event) == 0 &&
            event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
          callback_();
        }
      }
    }
  }

  gpiod_chip* chip_;       // GPIO chip handle
  gpiod_line* line_{};     // GPIO line handle
  unsigned int line_num_;  // Line number
  bool is_output_;         // Output mode flag

  std::atomic<bool> running_{false};  // Interrupt thread flag
  std::thread interrupt_thread_;      // Interrupt handler thread
  Callback callback_;                 // Registered callback
};
