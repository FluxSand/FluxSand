#pragma once

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "bsp_gpio.hpp"

/**
 * SPI device interface for configuration and register operations.
 */
class SpiDevice {
 public:
  /**
   * Opens and configures the SPI device.
   *
   * @param device SPI device file path (e.g., "/dev/spidev0.0")
   * @param speed  Transfer speed in Hz
   * @param mode   SPI mode (e.g., SPI_MODE_0)
   */
  SpiDevice(const std::string& device, uint32_t speed, uint8_t mode)
      : fd_(open(device.c_str(), O_RDWR)), speed_(speed) {
    assert(!device.empty()); /* Ensure device string is valid */

    if (fd_ < 0) {
      std::perror("Failed to open SPI device");
    }

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
      close(fd_);
      std::perror("Failed to configure SPI device");
    }
  }

  /** Closes the SPI device. */
  ~SpiDevice() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  /**
   * Reads a register via SPI.
   *
   * @param cs  GPIO chip select (active low)
   * @param reg Register address (MSB set to 1 for read operation)
   * @return Read value
   */
  uint8_t ReadRegister(Gpio* cs, uint8_t reg) {
    assert(cs); /* Ensure GPIO object is valid */

    std::array<uint8_t, 2> tx_buf = {{static_cast<uint8_t>(reg | 0x80), 0}};
    std::array<uint8_t, 2> rx_buf = {{0, 0}};
    struct spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uint64_t>(tx_buf.data());
    transfer.rx_buf = reinterpret_cast<uint64_t>(rx_buf.data());
    transfer.len = tx_buf.size();
    transfer.speed_hz = speed_;
    transfer.bits_per_word = 8;

    cs->Write(0);
    usleep(10);
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      cs->Write(1);
      std::perror("SPI read failed");
    }
    cs->Write(1);

    return rx_buf[1];
  }

  /**
   * Writes a value to a register via SPI.
   *
   * @param cs    GPIO chip select
   * @param reg   Register address
   * @param value Value to write
   */
  void WriteRegister(Gpio* cs, uint8_t reg, uint8_t value) {
    assert(cs); /* Ensure GPIO object is valid */

    std::array<uint8_t, 2> tx_buf = {{reg, value}};
    struct spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uint64_t>(tx_buf.data());
    transfer.len = tx_buf.size();
    transfer.speed_hz = speed_;
    transfer.bits_per_word = 8;

    cs->Write(0);
    usleep(10);
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      cs->Write(1);
      std::perror("SPI write failed");
    }
    cs->Write(1);
  }

  void ReadRegisters(Gpio* cs, uint8_t reg, uint8_t* buffer, size_t length) {
    assert(cs);
    assert(buffer);

    std::vector<uint8_t> tx_buf(length + 1, 0);
    std::vector<uint8_t> rx_buf(length + 1, 0);
    tx_buf[0] = reg | 0x80;

    struct spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uint64_t>(tx_buf.data());
    transfer.rx_buf = reinterpret_cast<uint64_t>(rx_buf.data());
    transfer.len = tx_buf.size();
    transfer.speed_hz = speed_;
    transfer.bits_per_word = 8;

    cs->Write(0);
    usleep(10);
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      cs->Write(1);
      std::perror("SPI multiple read failed");
    }
    cs->Write(1);

    std::memcpy(buffer, rx_buf.data() + 1, length);
  }

  int Fd() const { return fd_; }

  uint32_t Speed() const { return speed_; }

 private:
  int fd_;         /* SPI file descriptor */
  uint32_t speed_; /* Transfer speed in Hz */
};
