#pragma once

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <iostream>
#include <thread>

#include "ads1115.hpp"
#include "aht20.hpp"
#include "bmp280.hpp"
#include "bsp_pwm.hpp"
#include "comp_ahrs.hpp"
#include "comp_gui.hpp"
#include "comp_inference.hpp"
#include "inference_handler.hpp"
#include "input_handler.hpp"
#include "max7219.hpp"
#include "mode_manager.hpp"
#include "sensor_manager.hpp"

// The main application class for FluxSand, handling GUI updates,
// sensor integration, input processing, and mode-based logic.
class FluxSand {
 public:
  // Constructor: initializes hardware components, sensor manager,
  // inference handler, and input handler.
  FluxSand(PWM* pwm_buzzer, Gpio* gpio_user_button_1, Gpio* gpio_user_button_2,
           CompGuiX* gui, Bmp280* bmp280, Aht20* aht20, Ads1115<2>* ads1115,
           AHRS* ahrs, InferenceEngine* inference)
      : pwm_buzzer_(pwm_buzzer),
        gpio_user_button_1_(gpio_user_button_1),
        gpio_user_button_2_(gpio_user_button_2),
        gui_(gui),
        bmp280_(bmp280),
        aht20_(aht20),
        ads1115_(ads1115),
        ahrs_(ahrs),
        inference_(inference) {
    // Wait for system stabilization or hardware warm-up
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    // Prepare the buzzer
    pwm_buzzer_->SetDutyCycle(0.0f);
    pwm_buzzer_->Enable();

    // Initialize sensor manager with sensor instances and GUI
    sensor_manager_.Init(ads1115_, aht20_, bmp280_, gui_);

    // Initialize inference handler with inference engine and mode manager
    // and bind timer control callbacks
    inference_handler_.Init(
        inference_, &mode_manager_, gui_, pwm_buzzer_,
        [this](int duration) { StartTimer(duration); },
        [this]() { StopTimer(); });

    // Initialize input handler with buttons and callbacks for stopwatch/timer
    input_handler_.Init(
        gpio_user_button_1_, gpio_user_button_2_, pwm_buzzer_, gui_,
        &mode_manager_,
        [this]() {
          if (mode_manager_.IsStopwatchRunning())
            StopStopwatch();
          else
            StartStopwatch();
        },
        [this]() { StopTimer(); });
  }

  // Main update loop called repeatedly to render GUI based on current mode
  void Run() {
    // Get current system time
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&t);
    int hour = local_time->tm_hour;
    int minute = local_time->tm_min;

    // Update GUI with the current tilt angle
    gui_->SetGravityDegree(ahrs_->eulr_.rol);

    // Get current UI mode and orientation
    auto mode = mode_manager_.GetMode();
    bool landscape = mode_manager_.IsLandscape();

    switch (mode) {
      case ModeManager::Mode::TIME:
        // Render clock in portrait or landscape orientation
        if (landscape)
          gui_->RenderTimeLandscape(hour, minute);
        else
          gui_->RenderTimePortrait(hour, minute);
        break;

      case ModeManager::Mode::HUMIDITY:
        // Display humidity reading
        gui_->RenderHumidity(
            static_cast<uint8_t>(sensor_manager_.GetHumidity()));
        break;

      case ModeManager::Mode::TEMPERATURE:
        // Display compensated temperature, offset for calibration
        gui_->RenderTemperature(static_cast<uint8_t>(
            sensor_manager_.GetCompensatedTemperature()));
        break;

      case ModeManager::Mode::STOPWATCH: {
        // Display stopwatch time with a 100-minute limit
        int64_t display_sec = mode_manager_.GetStopwatchSeconds();
        if (display_sec >= 100 * 60 - 1) display_sec = 100 * 60 - 1;
        gui_->RenderTimeLandscape(display_sec / 60, display_sec % 60);
        break;
      }

      case ModeManager::Mode::TIMER: {
        // Handle timer countdown and animation
        int64_t remaining = mode_manager_.GetRemainingTimerSeconds();

        // If timer finished, trigger buzzer and disable sand animation
        if (remaining == 0 && mode_manager_.IsTimerRunning()) {
          pwm_buzzer_->PlayNote(PWM::NoteName::C, 8, 1000);
          gui_->SandDisable();
        }

        if (landscape) {
          // Show remaining time in landscape format
          gui_->RenderTimeLandscapeMS(remaining / 60, remaining % 60);
        } else if (mode_manager_.IsTimerRunning()) {
          // In portrait mode, animate sand flow
          gui_->SandEnable();
          int target_grid_down_count =
              128 - static_cast<int>(128.0f * static_cast<float>(remaining) /
                                     static_cast<float>(
                                         mode_manager_.GetMaxTimerDuration()));
          target_grid_down_count = std::clamp(target_grid_down_count, 0, 128);

          // Only move sand if more sand needs to fall
          if (target_grid_down_count > gui_->grid_down_.Count()) {
            SandGrid::MoveSand(&gui_->grid_up_, &gui_->grid_down_,
                               gui_->gravity_deg_);
          }
        } else {
          // If timer not running, show static time
          gui_->RenderTimePortraitMS(remaining / 60, remaining % 60);
        }
        break;
      }

      default:
        std::cout << "Unknown mode\n";
    }

    // Small delay to prevent high CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

 private:
  // Stopwatch control functions
  void StartStopwatch() { mode_manager_.StartStopwatch(); }
  void StopStopwatch() { mode_manager_.StopStopwatch(); }

  // Timer control functions
  void StartTimer(int duration_sec) {
    mode_manager_.StartTimer(duration_sec);
    gui_->Reset();
  }
  void StopTimer() { mode_manager_.StopTimer(); }

  // Hardware and component pointers
  PWM* pwm_buzzer_;
  Gpio* gpio_user_button_1_;
  Gpio* gpio_user_button_2_;
  CompGuiX* gui_;
  Bmp280* bmp280_;
  Aht20* aht20_;
  Ads1115<2>* ads1115_;
  AHRS* ahrs_;
  InferenceEngine* inference_;

  // Application-level modules
  ModeManager mode_manager_;
  SensorManager sensor_manager_;
  InferenceHandler inference_handler_;
  InputHandler input_handler_;
};
