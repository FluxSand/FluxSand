#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "bsp_i2c.hpp"  // Contains I2cDevice class implementation

/**
 * BMP280 temperature and pressure sensor driver
 * Supports I2C communication and hardware compensation
 */
class Bmp280 {
 public:
  /** Default I2C address (0x76 or 0x77) */
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x77;

  /** Sensor register addresses */
  enum Register : uint8_t {
    REG_ID = 0xD0,         // Chip ID register
    REG_RESET = 0xE0,      // Reset register
    REG_CTRL_MEAS = 0xF4,  // Measurement control
    REG_CONFIG = 0xF5,     // Configuration
    REG_PRESS_MSB = 0xF7,  // Pressure MSB
    REG_TEMP_MSB = 0xFA,   // Temperature MSB
    REG_CALIB = 0x88       // Calibration data start
  };

  /**
   * Initialize BMP280 with specified I2C device
   * @param i2c Pre-configured I2C device instance
   */
  Bmp280(I2cDevice& i2c) : i2c_(i2c) {
    uint8_t id = i2c_.ReadRegister(REG_ID);
    if (id != 0x58) {  // Verify chip ID
      throw std::runtime_error("Invalid BMP280 ID: 0x" + std::to_string(id));
    }

    ReadCalibration();
    Configure();

    thread_ = std::thread(&Bmp280::ThreadFun, this);
  }

  void ThreadFun() {
    while (true) {
      ReadTemperature();
      ReadPressure();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void Display() {
    std::cout << "Temperature: " << ReadTemperature() << " °C\n";
    std::cout << "Pressure: " << ReadPressure() / 100.0f << " hPa\n";
  }

  /**
   * Read compensated temperature in Celsius
   * @return Temperature in °C with 0.01°C resolution
   */
  float ReadTemperature() {
    int32_t adc_t = ReadRawTemperature();
    return static_cast<float>(CompensateTemperature(adc_t)) / 100.0f;
  }

  /**
   * Read compensated pressure in Pascals
   * @return Pressure in Pa with 0.01Pa resolution
   */
  float ReadPressure() {
    int32_t adc_p = ReadRawPressure();
    ReadRawTemperature();           // Required to update t_fine
    CompensateTemperature(adc_t_);  // Ensure t_fine is current
    return static_cast<float>(CompensatePressure(adc_p)) / 256.0f;
  }

 private:
  // NOLINTNEXTLINE
  I2cDevice& i2c_;
  int32_t t_fine_ = 0;  // Shared temperature compensation value

  // Calibration parameters
  uint16_t dig_t1_;
  int16_t dig_t2_, dig_t3_;
  uint16_t dig_p1_;
  int16_t dig_p2_, dig_p3_, dig_p4_, dig_p5_, dig_p6_, dig_p7_, dig_p8_,
      dig_p9_;

  int32_t adc_t_ = 0;  // Cached raw temperature value

  std::thread thread_;

  /** Configure sensor operating mode */
  void Configure() {
    // Set oversampling: temp x1, pressure x1, normal mode
    i2c_.WriteRegister(REG_CTRL_MEAS, 0b00100111);
    // Set standby time 0.5ms, filter off
    i2c_.WriteRegister(REG_CONFIG, 0b00000000);
  }

  /** Read and store calibration data from sensor */
  void ReadCalibration() {
    uint8_t buf[24];
    i2c_.ReadRegisters(REG_CALIB, buf, 24);

    // Temperature calibration
    dig_t1_ = static_cast<uint16_t>((buf[1] << 8) | buf[0]);
    dig_t2_ = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    dig_t3_ = static_cast<int16_t>((buf[5] << 8) | buf[4]);

    // Pressure calibration
    dig_p1_ = (buf[7] << 8) | buf[6];
    dig_p2_ = static_cast<int16_t>((buf[9] << 8) | buf[8]);
    dig_p3_ = static_cast<int16_t>((buf[11] << 8) | buf[10]);
    dig_p4_ = static_cast<int16_t>((buf[13] << 8) | buf[12]);
    dig_p5_ = static_cast<int16_t>((buf[15] << 8) | buf[14]);
    dig_p6_ = static_cast<int16_t>((buf[17] << 8) | buf[16]);
    dig_p7_ = static_cast<int16_t>((buf[19] << 8) | buf[18]);
    dig_p8_ = static_cast<int16_t>((buf[21] << 8) | buf[20]);
    dig_p9_ = static_cast<int16_t>((buf[23] << 8) | buf[22]);
  }

  /** Read raw temperature ADC value */
  int32_t ReadRawTemperature() {
    uint8_t data[3];
    i2c_.ReadRegisters(REG_TEMP_MSB, data, 3);
    adc_t_ = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    return adc_t_;
  }

  /** Read raw pressure ADC value */
  int32_t ReadRawPressure() {
    uint8_t data[3];
    i2c_.ReadRegisters(REG_PRESS_MSB, data, 3);
    return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  }

  /** Temperature compensation algorithm */
  int32_t CompensateTemperature(int32_t adc_T) {
    int32_t var1 = (((adc_T >> 3) - (static_cast<int32_t>(dig_t1_) << 1)) *
                        (static_cast<int32_t>(dig_t2_)) >>
                    11);

    int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(dig_t1_)) *
                      ((adc_T >> 4) - static_cast<int32_t>(dig_t1_))) >>
                     12) *
                        (static_cast<int32_t>(dig_t3_)) >>
                    14);

    t_fine_ = var1 + var2;
    return (t_fine_ * 5 + 128) >> 8;
  }

  /** Pressure compensation algorithm */
  uint32_t CompensatePressure(int32_t adc_P) {
    int64_t var1 = static_cast<int64_t>(t_fine_) - 128000;
    int64_t var2 = var1 * var1 * static_cast<int64_t>(dig_p6_);
    var2 += (var1 * static_cast<int64_t>(dig_p5_)) << 17;
    var2 += (static_cast<int64_t>(dig_p4_)) << 35;

    var1 = ((var1 * var1 * static_cast<int64_t>(dig_p3_)) >> 8) +
           ((var1 * static_cast<int64_t>(dig_p2_)) << 12);
    var1 = (((static_cast<int64_t>(1) << 47) + var1) *
            static_cast<int64_t>(dig_p1_)) >>
           33;

    if (var1 == 0) {
      return 0;  // Prevent division by zero
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (static_cast<int64_t>(dig_p9_) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(dig_p8_) * p) >> 19;

    return static_cast<uint32_t>((p + var1 + var2) >> 8) +
           (static_cast<int64_t>(dig_p7_) << 4);
  }
};
