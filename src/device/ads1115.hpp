#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <thread>

#include "bsp_gpio.hpp"
#include "bsp_i2c.hpp"

/**
 * @brief ADS1115 ADC driver with GPIO interrupt-based channel cycling
 *
 * Continuously scans 4 single-ended channels using ALERT/RDY pin as data-ready
 * signal, stores voltage values in `voltages_[]`, and allows user-defined
 * per-channel callbacks.
 */
template <int CHANNEL_NUM>
class Ads1115 {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x48;
  static constexpr uint8_t POINTER_CONVERSION = 0x00;
  static constexpr uint8_t POINTER_CONFIG = 0x01;

  /**
   * @brief Constructor
   * @param i2c      I2C device reference
   * @param drdy     GPIO for ALERT/RDY pin (interrupt-enabled)
   * @param address  I2C address (default 0x48)
   */
  Ads1115(I2cDevice& i2c, Gpio& drdy, uint8_t address = DEFAULT_I2C_ADDR)
      : i2c_(i2c), drdy_gpio_(drdy), i2c_addr_(address) {
    EnableReadyInterruptMode();
    StartScan();
  }

  /**
   * @brief Starts scanning the channels and enabling data-ready interrupt
   */
  void StartScan() {
    scanning_ = true;
    current_channel_ = 0;
    ConfigureChannel(current_channel_);

    drdy_gpio_.EnableInterruptRisingEdgeWithCallback(
        [this]() { OnDataReady(); });
  }

  /**
   * @brief Stops the scan process
   */
  void Stop() { scanning_ = false; }

  /**
   * @brief Returns the latest voltage reading from a specific channel
   * @param channel  Index (0 ~ CHANNEL_NUM-1)
   * @return Voltage value in volts
   */
  float GetVoltage(int channel) const {
    if (channel < 0 || channel >= CHANNEL_NUM) {
      return 0.0f;
    }
    return voltages_[channel];
  }

  /**
   * @brief Registers interrupt-based data-ready mode (comparator configuration)
   */
  void EnableReadyInterruptMode() {
    // Set Lo_thresh = 0x0000 (MSB = 0)
    uint8_t lo_thresh[3] = {0x02, 0x00, 0x00};
    i2c_.WriteRaw(lo_thresh, 3);

    // Set Hi_thresh = 0x8000 (MSB = 1)
    uint8_t hi_thresh[3] = {0x03, 0x80, 0x00};
    i2c_.WriteRaw(hi_thresh, 3);
  }

  /**
   * @brief Prints voltage readings of all channels to console
   */
  void Display() const {
    for (int i = 0; i < CHANNEL_NUM; ++i) {
      printf("Channel %d: %.4f V  ", i, voltages_[i]);
    }
    printf("\n");
  }

  /**
   * @brief Registers a callback function for a specific channel
   * @param channel  Index (0 ~ CHANNEL_NUM-1)
   * @param cb       Callback function taking float voltage as parameter
   */
  void RegisterChannelCallback(int channel, std::function<void(float)> cb) {
    if (channel >= 0 && channel < CHANNEL_NUM) {
      callbacks_[channel] = cb;
    }
  }

 private:
  // NOLINTNEXTLINE
  I2cDevice& i2c_;
  // NOLINTNEXTLINE
  Gpio& drdy_gpio_;
  uint8_t i2c_addr_;
  std::function<void(float)> callbacks_[CHANNEL_NUM];

  std::atomic<bool> scanning_{false};
  int current_channel_ = 0;
  float voltages_[CHANNEL_NUM] = {0.0f};

  /**
   * @brief Called on rising edge of ALERT/RDY pin
   * Reads current value, stores it, calls callback, then switches to next
   * channel
   */
  void OnDataReady() {
    if (!scanning_) {
      return;
    }

    // Read and convert voltage
    int16_t raw = ReadConversion();
    voltages_[current_channel_] =
        static_cast<float>(raw) * 0.000125f;  // 125 µV/LSB

    // Invoke user callback
    if (callbacks_[current_channel_]) {
      callbacks_[current_channel_](voltages_[current_channel_]);
    }

    // Switch to next channel
    current_channel_ = (current_channel_ + 1) % CHANNEL_NUM;
    ConfigureChannel(current_channel_);
  }

  /**
   * @brief Configures ADS1115 for a specific single-ended channel
   * @param channel Index 0~3
   */
  void ConfigureChannel(int channel) {
    if (channel < 0 || channel > 3) {
      return;
    }

    uint16_t config = 0;
    config |= (0x04 + channel) << 12;  // MUX: AINx vs GND
    config |= (1 << 9);                // PGA ±4.096V
    config |= (0 << 8);                // Mode: Continuous
    config |= (0b111 << 5);            // Data Rate: 860 SPS

    uint8_t data[3] = {
        POINTER_CONFIG,
        static_cast<uint8_t>((config >> 8) & 0xFF),
        static_cast<uint8_t>(config & 0xFF),
    };

    i2c_.WriteRaw(data, 3);

    // Wait for config to take effect; discard first reading
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ReadConversion();
  }

  /**
   * @brief Reads the 16-bit signed conversion result
   * @return ADC result (LSB = 125 µV at ±4.096V)
   */
  int16_t ReadConversion() {
    uint8_t pointer = POINTER_CONVERSION;
    i2c_.WriteRaw(&pointer, 1);

    uint8_t buf[2] = {};
    i2c_.ReadRegisters(POINTER_CONVERSION, buf, 2);

    return static_cast<int16_t>((buf[0] << 8) | buf[1]);
  }
};
