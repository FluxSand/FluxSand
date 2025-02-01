#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <format>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "comp_type.hpp"
#include "gpio.hpp"
#include "spi.hpp"

namespace imu {

/**
 * MPU9250 class
 *
 * Provides initialization and data retrieval for MPU9250 via SPI.
 */
class Mpu9250 {
 public:
  /**
   * Constructor
   *
   * @param spi_device Pointer to an SPI device object
   * @param gpio_cs    Pointer to a GPIO object (chip select)
   */
  Mpu9250(spi::SpiDevice* spi_device, gpio::Gpio* gpio_cs)
      : spi_device_(spi_device), gpio_cs_(gpio_cs), accel_(), gyro_(), mag_() {
    assert(spi_device_ && gpio_cs_);
  }

  /**
   * Initializes MPU9250 settings.
   *
   * Configures power management, digital low-pass filter, sensor ranges,
   * I2C Master mode for the magnetometer, and performs a WHO_AM_I check.
   */
  void Initialize() {
    /* Reset and wake up the MPU9250 */
    spi_device_->WriteRegister(gpio_cs_, PWR_MGMT_1, 0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Read and verify WHO_AM_I register */
    uint8_t who_am_i = spi_device_->ReadRegister(gpio_cs_, WHO_AM_I);
    std::cout << std::format("MPU9250 WHO_AM_I: 0x{:02X}\n", who_am_i);

    if (who_am_i != 0x71 && who_am_i != 0x68 && who_am_i != 0x70) {
      throw std::runtime_error(std::format(
          "Error: MPU9250 connection failed (WHO_AM_I: 0x{:02X})", who_am_i));
    }

    /* Configure I2C Master mode for communication with AK8963 (magnetometer).
     */
    spi_device_->WriteRegister(gpio_cs_, INT_PIN_CFG, 0x30);

    /* Set I2C master mode with a clock speed of 400 kHz. */
    spi_device_->WriteRegister(gpio_cs_, I2C_MST_CTRL, 0x4D);

    /* Enable I2C Master mode by setting the I2C_MST_EN bit in USER_CTRL. */
    spi_device_->WriteRegister(gpio_cs_, USER_CTRL, 0x20);

    /* Enable delay for I2C Slave 0 transfers. */
    spi_device_->WriteRegister(gpio_cs_, I2C_MST_DELAY_CTRL, 0x01);

    /* Enable I2C communication for external sensors. */
    spi_device_->WriteRegister(gpio_cs_, I2C_SLV0_CTRL, 0x81);

    /* Configure Digital Low-Pass Filter (DLPF). */
    spi_device_->WriteRegister(gpio_cs_, CONFIG, 1);

    /* Set the gyroscope sampling rate. */
    spi_device_->WriteRegister(gpio_cs_, SMPLRT_DIV, 0x07);

    /* Configure gyroscope full-scale range to ±2000°/s. */
    spi_device_->WriteRegister(gpio_cs_, GYRO_CONFIG, 0x1B);

    /* Configure accelerometer low-pass filter. */
    spi_device_->WriteRegister(gpio_cs_, ACCEL_CONFIG_2, 0x0F);

    /* Configure accelerometer full-scale range to ±16g. */
    spi_device_->WriteRegister(gpio_cs_, ACCEL_CONFIG, 0x18);

    /* Reset the AK8963 magnetometer. */
    WriteMagRegister(AK8963_CNTL2_REG, AK8963_CNTL2_SRST);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    /* Configure AK8963 magnetometer for continuous measurement mode 1. */
    WriteMagRegister(AK8963_CNTL1_REG, 0x12);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  /**
   * Reads sensor data for acceleration and gyroscope.
   */
  void ReadData() {
    std::array<uint8_t, 6> accel_data{};
    std::array<uint8_t, 6> gyro_data{};

    for (size_t i = 0; i < accel_data.size(); ++i) {
      accel_data[i] = spi_device_->ReadRegister(
          gpio_cs_, ACCEL_XOUT_H + static_cast<uint8_t>(i));
      gyro_data[i] = spi_device_->ReadRegister(
          gpio_cs_, GYRO_XOUT_H + static_cast<uint8_t>(i));
    }

    constexpr float ACCEL_SCALE = 16.0f / 32768.0f * 9.80665f;
    constexpr float GYRO_SCALE = 250.0f / 32768.0f * M_PI / 180.0f;

    accel_.x = static_cast<float>(
                   static_cast<int16_t>((accel_data[0] << 8) | accel_data[1])) *
               ACCEL_SCALE;
    accel_.y = static_cast<float>(
                   static_cast<int16_t>((accel_data[2] << 8) | accel_data[3])) *
               ACCEL_SCALE;
    accel_.z = static_cast<float>(
                   static_cast<int16_t>((accel_data[4] << 8) | accel_data[5])) *
               ACCEL_SCALE;

    gyro_.x = static_cast<float>(
                  static_cast<int16_t>((gyro_data[0] << 8) | gyro_data[1])) *
              GYRO_SCALE;
    gyro_.y = static_cast<float>(
                  static_cast<int16_t>((gyro_data[2] << 8) | gyro_data[3])) *
              GYRO_SCALE;
    gyro_.z = static_cast<float>(
                  static_cast<int16_t>((gyro_data[4] << 8) | gyro_data[5])) *
              GYRO_SCALE;
  }

  /**
   * Displays the most recently read sensor data.
   */
  void DisplayData() {
    std::cout << std::format(
        "Acceleration: X={} Y={} Z={} | Gyroscope: X={} Y={} Z={}\n", accel_.x,
        accel_.y, accel_.z, gyro_.x, gyro_.y, gyro_.z);
  }

 private:
  /**
   * Writes a value to the AK8963 magnetometer via I2C.
   *
   * @param reg   Magnetometer register address
   * @param value Value to write
   */
  void WriteMagRegister(uint8_t reg, uint8_t value) {
    spi_device_->WriteRegister(gpio_cs_, reg, value);
  }

  /** MPU9250 register addresses */
  static constexpr uint8_t WHO_AM_I = 0x75;
  static constexpr uint8_t PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t CONFIG = 0x1A;
  static constexpr uint8_t SMPLRT_DIV = 0x19;
  static constexpr uint8_t GYRO_CONFIG = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t ACCEL_CONFIG_2 = 0x1D;
  static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  static constexpr uint8_t GYRO_XOUT_H = 0x43;
  static constexpr uint8_t USER_CTRL = 0x6A;
  static constexpr uint8_t INT_PIN_CFG = 0x37;
  static constexpr uint8_t I2C_MST_CTRL = 0x24;
  static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
  static constexpr uint8_t I2C_SLV0_CTRL = 0x27;

  /** AK8963 Magnetometer registers */
  static constexpr uint8_t AK8963_CNTL1_REG = 0x0A;
  static constexpr uint8_t AK8963_CNTL2_REG = 0x0B;
  static constexpr uint8_t AK8963_CNTL2_SRST = 0x01;

  spi::SpiDevice* spi_device_; /**< SPI device handle */
  gpio::Gpio* gpio_cs_;        /**< GPIO chip select handle */

  Type::Vector3 accel_; /**< Accelerometer data */
  Type::Vector3 gyro_;  /**< Gyroscope data */
  Type::Vector3 mag_;   /**< Magnetometer data */
};

}  // namespace imu
