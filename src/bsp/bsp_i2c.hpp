#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * I2C device interface for configuration and register operations.
 */
class I2cDevice {
 public:
  /**
   * Opens and configures the I2C device.
   *
   * @param device I2C device file path (e.g., "/dev/i2c-1")
   * @param addr   7-bit I2C device address
   */
  I2cDevice(const std::string& device, uint8_t addr)
      : fd_(open(device.c_str(), O_RDWR)), addr_(addr) {
    assert(!device.empty()); /* Ensure device string is valid */

    if (fd_ < 0) {
      throw std::runtime_error("Failed to open I2C device: " + device);
    }

    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
      close(fd_);
      throw std::runtime_error("Failed to configure I2C address");
    }
  }

  /** Closes the I2C device. */
  ~I2cDevice() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  /**
   * Reads a single byte from an I2C register.
   *
   * @param reg Register address
   * @return Value read
   */
  uint8_t ReadRegister(uint8_t reg) {
    if (write(fd_, &reg, 1) != 1) {
      throw std::runtime_error("I2C write (set register) failed");
    }

    uint8_t value = 0;
    if (read(fd_, &value, 1) != 1) {
      throw std::runtime_error("I2C read failed");
    }

    return value;
  }

  /**
   * Writes a single byte to an I2C register.
   *
   * @param reg   Register address
   * @param value Value to write
   */
  void WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(fd_, buf, 2) != 2) {
      throw std::runtime_error("I2C write failed");
    }
  }

  /**
   * Reads multiple bytes starting from a given register.
   *
   * @param reg    Starting register address
   * @param buffer Destination buffer
   * @param length Number of bytes to read
   */
  void ReadRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    assert(buffer);

    if (write(fd_, &reg, 1) != 1) {
      throw std::runtime_error("I2C write (set start register) failed");
    }

    if (read(fd_, buffer, length) != static_cast<ssize_t>(length)) {
      throw std::runtime_error("I2C multi-byte read failed");
    }
  }

  /**
   * Writes raw bytes directly to the I2C device without a register prefix.
   *
   * @param data   Pointer to raw data buffer
   * @param length Number of bytes to write
   */
  void WriteRaw(const uint8_t* data, size_t length) {
    assert(data);

    if (write(fd_, data, length) != static_cast<ssize_t>(length)) {
      throw std::runtime_error("I2C raw write failed");
    }
  }

  /**
   * Returns the underlying file descriptor.
   */
  int Fd() const { return fd_; }

  /**
   * Returns the I2C slave address.
   */
  uint8_t Address() const { return addr_; }

 private:
  int fd_;       /* I2C file descriptor */
  uint8_t addr_; /* I2C device address */
};
