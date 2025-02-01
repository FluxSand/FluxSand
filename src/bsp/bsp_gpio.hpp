#pragma once

#include <cassert>
#include <stdexcept>
#include <string>

#include "bsp.hpp"

/**
 * Gpio class
 *
 * Handles GPIO initialization, configuration (input/output), and read/write
 * operations.
 */
class Gpio {
 public:
  /**
   * Constructor
   *
   * @param chip_name    GPIO chip name, e.g., "gpiochip0"
   * @param line_num     GPIO line number
   * @param is_output    true for output mode, false for input mode
   * @param default_value Initial value for output mode (default: 1)
   */
  Gpio(const std::string& chip_name, unsigned int line_num, bool is_output,
       int default_value = 1)
      : chip_(gpiod_chip_open_by_name(chip_name.c_str())),
        is_output_(is_output) {
    assert(!chip_name.empty());  // Ensure chip name is valid

    if (!chip_) {
      throw std::runtime_error("Failed to open GPIO chip: " + chip_name);
    }

    line_ = gpiod_chip_get_line(chip_, line_num);
    if (!line_) {
      gpiod_chip_close(chip_);
      throw std::runtime_error("Failed to get GPIO line: " +
                               std::to_string(line_num));
    }

    int ret = is_output_
                  ? gpiod_line_request_output(line_, nullptr, default_value)
                  : gpiod_line_request_input(line_, nullptr);
    if (ret < 0) {
      throw std::runtime_error("Failed to configure GPIO line: " +
                               std::to_string(line_num));
    }
  }

  /**
   * Destructor
   *
   * Releases allocated resources.
   */
  ~Gpio() {
    if (line_) {
      gpiod_line_release(line_);
    }

    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

  /**
   * Writes a value to the GPIO line (output mode only).
   *
   * @param value Value to write (0 or 1)
   */
  void Write(int value) {
    assert(is_output_);  // Ensure it's in output mode
    if (gpiod_line_set_value(line_, value) < 0) {
      throw std::runtime_error("GPIO write failed");
    }
  }

  /**
   * Reads the current value of the GPIO line.
   *
   * @return Current GPIO value (0 or 1)
   */
  int Read() const {
    int value = gpiod_line_get_value(line_);
    if (value < 0) {
      throw std::runtime_error("GPIO read failed");
    }
    return value;
  }

 private:
  gpiod_chip* chip_;   /* GPIO chip handle */
  gpiod_line* line_{}; /* GPIO line handle */
  bool is_output_;     /* Output mode flag */
};
