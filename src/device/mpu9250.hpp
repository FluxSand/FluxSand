#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "comp_type.hpp"
#include "message.hpp"

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
  Mpu9250(SpiDevice* spi_device, Gpio* gpio_cs)
      : spi_device_(spi_device),
        gpio_cs_(gpio_cs),
        accel_(),
        gyro_(),
        mag_(),
        accel_topic_("accel", true),
        gyro_topic_("gyro", true) {
    assert(spi_device_ && gpio_cs_);

    accel_topic_ = LibXR::Topic::CreateTopic<Type::Vector3>("accel");
    gyro_topic_ = LibXR::Topic::CreateTopic<Type::Vector3>("gyro");

    Initialize();
    LoadCalibrationData();
    main_thread_ = std::thread(&Mpu9250::MainThreadTask, this);
    calibrate_thread_ = std::thread(&Mpu9250::CalibrateThreadTask, this);
  }

  /**
   * Thread task that periodically reads and displays sensor data.
   */
  void MainThreadTask() {
    std::chrono::microseconds period_us(static_cast<int>(1000));
    std::chrono::steady_clock::time_point next_time =
        std::chrono::steady_clock::now();

    while (true) {
      ReadData();
      accel_topic_.Publish(accel_);
      gyro_topic_.Publish(gyro_);
      next_time += period_us;
      std::this_thread::sleep_until(next_time);
    }
  }

  void CalibrateThreadTask() {
    while (true) {
      auto start_time = std::chrono::steady_clock::now();
      std::chrono::microseconds period_us(static_cast<int>(1000));
      std::chrono::steady_clock::time_point next_time =
          std::chrono::steady_clock::now();
      uint32_t counter = 0;
      double gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
      static bool cali_done = false;

      if (cali_done) {
        std::this_thread::sleep_for(std::chrono::seconds(UINT32_MAX));
        continue;
      }

      while (true) {
        if (std::fabsf(gyro_delta_.x) > 0.005 || fabsf(gyro_delta_.y) > 0.005 ||
            fabsf(gyro_delta_.z) > 0.01) {
          next_time += period_us;
          std::this_thread::sleep_until(next_time);
          break;
        }

        auto now = std::chrono::steady_clock::now();
        if (now - start_time > std::chrono::seconds(5) &&
            now - start_time < std::chrono::seconds(30)) {
          counter++;
          gyro_offset_x += gyro_.x;
          gyro_offset_y += gyro_.y;
          gyro_offset_z += gyro_.z;
        }

        if (now - start_time > std::chrono::seconds(35) && cali_done == false) {
          float bias_x =
              static_cast<float>(gyro_offset_x / static_cast<double>(counter));
          float bias_y =
              static_cast<float>(gyro_offset_y / static_cast<double>(counter));
          float bias_z =
              static_cast<float>(gyro_offset_z / static_cast<double>(counter));

          if (std::fabsf(bias_x) > 0.005 || std::fabsf(bias_y) > 0.005 ||
              std::fabsf(bias_z) > 0.005) {
            gyro_bias_.x += bias_x;
            gyro_bias_.y += bias_y;
            gyro_bias_.z += bias_z;
            SaveCalibrationData();
            std::cout << "Calibration data saved\n";
          } else {
            std::cout << "No need to calibrate\n";
          }
          cali_done = true;
          std::cout << "Calibration completed\n";
        }

        next_time += period_us;
        std::this_thread::sleep_until(next_time);
      }
    }
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
    std::cout << std::format("MPU9250 initialized. WHO_AM_I: 0x{:02X}\n",
                             who_am_i);

    if (who_am_i != 0x71 && who_am_i != 0x68 && who_am_i != 0x70) {
      throw std::runtime_error(std::format(
          "Error: MPU9250 connection failed (WHO_AM_I: 0x{:02X})", who_am_i));
    }

    /* Gyroscope clock source configuration */
    spi_device_->WriteRegister(gpio_cs_, PWR_MGMT_1, 0x03);

    /* Enable Accelerometer and Gyroscope */
    spi_device_->WriteRegister(gpio_cs_, PWR_MGMT_2, 0x00);

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
    spi_device_->WriteRegister(gpio_cs_, CONFIG, 3);

    /* Set the gyroscope sampling rate. */
    spi_device_->WriteRegister(gpio_cs_, SMPLRT_DIV, 0x01);

    /* Configure gyroscope full-scale range to ±2000°/s. */
    spi_device_->WriteRegister(gpio_cs_, GYRO_CONFIG, 0x18);

    /* Configure accelerometer full-scale range to ±16g. */
    spi_device_->WriteRegister(gpio_cs_, ACCEL_CONFIG, 0x18);

    /* Configure accelerometer low-pass filter. */
    spi_device_->WriteRegister(gpio_cs_, ACCEL_CONFIG_2, 0x00);

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
    std::array<uint8_t, 14> data{};

    spi_device_->ReadRegisters(gpio_cs_, ACCEL_XOUT_H, &data[0], 14);

    constexpr float ACCEL_SCALE = 16.0f / 32768.0f * 9.80665f;
    constexpr float GYRO_SCALE = 2000.0f / 32768.0f * M_PI / 180.0f;

    uint8_t* accel_data = &data[0];
    uint8_t* temperature_data = &data[6];
    uint8_t* gyro_data = &data[8];

    accel_.x = static_cast<float>(
                   static_cast<int16_t>((accel_data[0] << 8) | accel_data[1])) *
               ACCEL_SCALE;
    accel_.y = static_cast<float>(
                   static_cast<int16_t>((accel_data[2] << 8) | accel_data[3])) *
               ACCEL_SCALE;
    accel_.z = static_cast<float>(
                   static_cast<int16_t>((accel_data[4] << 8) | accel_data[5])) *
               ACCEL_SCALE;

    temperature_ = static_cast<float>(static_cast<int16_t>(
                       (temperature_data[0] << 8) | temperature_data[1])) /
                       333.87f +
                   21.0f;

    float gyro_x = static_cast<float>(static_cast<int16_t>((gyro_data[0] << 8) |
                                                           gyro_data[1])) *
                       GYRO_SCALE -
                   gyro_bias_.x;
    float gyro_y = static_cast<float>(static_cast<int16_t>((gyro_data[2] << 8) |
                                                           gyro_data[3])) *
                       GYRO_SCALE -
                   gyro_bias_.y;
    float gyro_z = static_cast<float>(static_cast<int16_t>((gyro_data[4] << 8) |
                                                           gyro_data[5])) *
                       GYRO_SCALE -
                   gyro_bias_.z;

    gyro_delta_.x = gyro_x - gyro_.x;
    gyro_delta_.y = gyro_y - gyro_.y;
    gyro_delta_.z = gyro_z - gyro_.z;

    gyro_.x = gyro_x;
    gyro_.y = gyro_y;
    gyro_.z = gyro_z;
  }

  /**
   * Displays the most recently read sensor data.
   */
  void DisplayData() {
    float accel_instensity =
        sqrt(accel_.x * accel_.x + accel_.y * accel_.y + accel_.z * accel_.z);
    std::cout << std::format(
        "Acceleration: [X={:+.4f}, Y={:+.4f}, Z={:+.4f} | Intensity={:+.4f}] | "
        "Gyroscope: "
        "[X={:+.4f}, Y={:+.4f}, Z={:+.4f}] | Temperature: {:+.4f} °C\n",
        accel_.x, accel_.y, accel_.z, accel_instensity, gyro_.x, gyro_.y,
        gyro_.z, temperature_);
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

  /**
   * Save the calibration data to a file.
   */
  void SaveCalibrationData() {
    std::ofstream file("cali_data.bin", std::ios::binary);
    if (!file) {
      std::cerr << "Error: Unable to open cali_data.bin for writing.\n";
      return;
    }
    file.write(reinterpret_cast<const char*>(&gyro_bias_), sizeof(gyro_bias_));
    file.close();
    std::cout << "Calibration data saved successfully.\n";
  }

  /**
   * Load the calibration data from a file.
   */
  void LoadCalibrationData() {
    std::ifstream file("cali_data.bin", std::ios::in | std::ios::binary);
    if (!file) {
      std::cerr
          << "MPU9250 calibration file not found. Using default values.\n";
      return;
    }

    // Check file size
    file.seekg(0, std::ios::end);
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file_size != sizeof(gyro_bias_)) {
      std::cerr
          << "Error: MPU9250 calibration file size mismatch. Using default "
             "values.\n";
      file.close();
      gyro_bias_ = {0, 0, 0};
      return;
    }

    // Read data
    file.read(reinterpret_cast<char*>(&gyro_bias_), sizeof(gyro_bias_));
    file.close();

    // Validate data: Ensure gyro_bias values are within a reasonable range
    if (std::abs(gyro_bias_.x) > 1.0f || std::abs(gyro_bias_.y) > 1.0f ||
        std::abs(gyro_bias_.z) > 1.0f || std::isnan(gyro_bias_.x) ||
        std::isnan(gyro_bias_.y) || std::isnan(gyro_bias_.z) ||
        std::isinf(gyro_bias_.x) || std::isinf(gyro_bias_.y) ||
        std::isinf(gyro_bias_.z)) {
      std::cerr
          << "Error: MPU9250 invalid calibration data detected. Resetting to "
             "default values.\n";
      gyro_bias_ = {0, 0, 0};
    } else {
      std::cout << "MPU9250 calibration data loaded successfully: "
                << "X=" << gyro_bias_.x << ", "
                << "Y=" << gyro_bias_.y << ", "
                << "Z=" << gyro_bias_.z << "\n";
    }

    std::cout << '\n';
  }

  /** MPU9250 register addresses */
  static constexpr uint8_t WHO_AM_I = 0x75;
  static constexpr uint8_t PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t PWR_MGMT_2 = 0x6C;
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

  SpiDevice* spi_device_; /* SPI device handle */
  Gpio* gpio_cs_;         /* GPIO chip select handle */

  Type::Vector3 accel_;      /* Accelerometer data */
  Type::Vector3 gyro_;       /* Gyroscope data */
  Type::Vector3 mag_;        /* Magnetometer data */
  Type::Vector3 gyro_delta_; /* Gyroscope delta data */

  Type::Vector3 gyro_bias_ = {0, 0, 0}; /* Gyroscope calibration data */

  float temperature_ = 0; /* Temperature data */

  LibXR::Topic accel_topic_; /* Accelerometer topic */
  LibXR::Topic gyro_topic_;  /* Gyroscope topic */

  std::thread main_thread_, calibrate_thread_; /* Thread */
};
