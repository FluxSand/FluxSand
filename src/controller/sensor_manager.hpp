#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>

#include "ads1115.hpp"
#include "aht20.hpp"
#include "bmp280.hpp"
#include "comp_gui.hpp"

// Manages environmental sensors and signal processing for temperature,
// humidity, pressure, and ambient light. Also handles GUI brightness updates.
class SensorManager {
 public:
  // Initialize the sensor manager with connected sensor instances and GUI.
  void Init(Ads1115<2>* ads, Aht20* aht, Bmp280* bmp, CompGuiX* gui) {
    aht_ = aht;
    bmp_ = bmp;
    gui_ = gui;

    // Register callback for channel 0: NTC thermistor (external temperature)
    ads->RegisterChannelCallback(0, [this](float voltage) {
      constexpr float VCC = 3.3f;         // Supply voltage
      constexpr float R_REF = 100000.0f;  // Reference resistor value (Ω)
      float r_ntc =
          R_REF * voltage / (VCC - voltage);  // Calculate NTC resistance

      // Steinhart-Hart approximation constants for thermistor
      constexpr float B = 3950.0f;    // B-value of thermistor
      constexpr float T0 = 298.15f;   // Reference temperature (Kelvin)
      constexpr float R0 = 10000.0f;  // Reference resistance at T0

      // Convert resistance to temperature in Celsius
      float temp = 1.0f / (1.0f / T0 + (1.0f / B) * log(r_ntc / R0)) - 273.15f;
      temperature_ = temp;
    });

    // Register callback for channel 1: Photodiode (ambient light sensor)
    ads->RegisterChannelCallback(1, [this](float voltage) {
      constexpr float VCC = 3.3f;
      constexpr float R_REF = 100000.0f;
      float r_photo =
          R_REF * voltage / (VCC - voltage);  // Calculate resistance

      // Empirical constants for light sensor calibration
      constexpr float K = 1500000.0f;
      constexpr float GAMMA = 1.5f;
      float lux = K / pow(r_photo, GAMMA);  // Convert resistance to lux

      // Maintain a moving average with a deque of last 50 readings
      light_queue_.push_front(lux);
      if (light_queue_.size() > 50) {
        float avg = 0.0f;
        for (auto l : light_queue_) avg += l;
        avg /= light_queue_.size();
        light_ = avg;

        // Reduce GUI update rate using a counter
        static int counter = 0;
        if (++counter > 5) {
          if (gui_) gui_->SetLight(static_cast<uint8_t>(light_ / 20 + 1));
          counter = 0;
        }

        // Keep the queue size within limit
        light_queue_.pop_back();
      }
    });
  }

  // Accessor for external thermistor temperature (processed via ADS1115).
  float GetTemperature() const { return temperature_; }

  // Accessor for smoothed light level in lux.
  float GetLight() const { return light_; }

  // Accessor for humidity from AHT20 sensor.
  float GetHumidity() const { return aht_ ? aht_->GetHumidity() : 0.0f; }

  // Accessor for ambient temperature from AHT20 (used for compensation).
  float GetCompensatedTemperature() const {
    return aht_ ? aht_->GetTemperature() : 0.0f;
  }

  // Accessor for barometric pressure from BMP280 sensor.
  float GetPressure() const { return bmp_ ? bmp_->ReadPressure() : 0.0f; }

 private:
  // Sensor and GUI component references
  Aht20* aht_ = nullptr;
  Bmp280* bmp_ = nullptr;
  CompGuiX* gui_ = nullptr;

  // Cached sensor values
  float temperature_ = 0.0f;  // External thermistor temperature
  float light_ = 0.0f;        // Smoothed ambient light in lux

  // Queue for moving average light smoothing
  std::deque<float> light_queue_;
};
