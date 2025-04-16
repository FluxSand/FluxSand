#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "bsp_i2c.hpp"  // I2cDevice class definition

/* AHT20 temperature and humidity sensor driver */
/* Supports I2C communication and periodic measurement */
class Aht20 {
 public:
  /* Default I2C address of AHT20 (0x38) */
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x38;

  /**
   * Constructor: initialize the sensor and start the measurement thread
   *
   * @param i2c Reference to an I2C device
   */
  explicit Aht20(I2cDevice& i2c) : i2c_(i2c) {
    InitSensor();
    thread_ = std::thread(&Aht20::ThreadFun, this);
  }

  /**
   * Destructor: stop the background thread
   */
  ~Aht20() {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  /**
   * Print current temperature and humidity to stdout
   */
  void Display() {
    std::cout << "Temperature: " << temperature_ << " °C" << '\n';
    std::cout << "Humidity: " << humidity_ << " %RH" << '\n';
  }

  /* Get the current temperature value (unit: Celsius) */
  float GetTemperature() const { return temperature_; }

  /* Get the current humidity value (unit: %RH) */
  float GetHumidity() const { return humidity_; }

 private:
  I2cDevice& i2c_;     /* Reference to I2C device */
  std::thread thread_; /* Background measurement thread */
  std::atomic<bool> running_{true};

  float temperature_ = 0.0f; /* Current temperature */
  float humidity_ = 0.0f;    /* Current humidity */

  /**
   * Initialize the AHT20 sensor
   */
  void InitSensor() {
    /* Write initialization sequence */
    i2c_.WriteRegister(0xBE, 0x08);
    i2c_.WriteRegister(0xBE, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  /**
   * Thread function: read temperature and humidity every 500ms
   */
  void ThreadFun() {
    while (running_) {
      ReadSensor();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  /**
   * Read and decode one sample of temperature and humidity
   */
  void ReadSensor() {
    const uint8_t CMD[3] = {0xAC, 0x33, 0x00};
    i2c_.WriteRaw(CMD, 3); /* Send measurement command */

    std::this_thread::sleep_for(
        std::chrono::milliseconds(80)); /* Wait for conversion */

    uint8_t buf[6] = {};
    i2c_.ReadRegisters(0x00, buf, 6); /* Read 6-byte measurement data */

    if ((buf[0] & 0x80) != 0) {
      return; /* Sensor is busy */
    }

    /* Parse humidity */
    uint32_t raw_h = ((buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4));
    float humidity = static_cast<float>(raw_h) * 100.0f / 1048576.0f;

    if (humidity != 0.0f) {
      humidity_ = humidity;
    }

    /* Parse temperature */
    uint32_t raw_t = ((buf[3] & 0x0F) << 16) | (buf[4] << 8) | buf[5];
    temperature_ = static_cast<float>(raw_t) * 200.0f / 1048576.0f - 50.0f;
  }
};
