#pragma once

#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

/**
 * @brief PWM driver with Beep and PlayNote functionality.
 *
 * Designed for Raspberry Pi 5, using GPIO12/13.
 *
 * ⚠️ Requires enabling the following overlay in `/boot/firmware/config.txt`:
 *     dtoverlay=pwm-2chan
 *
 * GPIO12 = PWM channel 0, GPIO13 = PWM channel 1.
 */
class PWM {
 public:
  // Musical note names used in PlayNote().
  // NOLINTNEXTLINE
  enum class NoteName { C = 0, Cs, D, Ds, E, F, Fs, G, Gs, A, As, B };

  /**
   * @brief Constructor: initialize PWM channel and set default frequency/duty.
   * @param channel PWM channel number (0 or 1).
   * @param frequency_hz Initial frequency in Hz (default: 1000).
   * @param duty_percent Initial duty cycle as a percentage (0.0 to 1.0).
   * @param chip PWM chip number (default: 2 for RPi 5).
   */
  PWM(int channel, int frequency_hz = 1000, float duty_percent = 0.0f,
      int chip = 2)
      : channel_(channel), period_ns_(static_cast<int>(1e9 / frequency_hz)) {
    chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    pwm_path_ = chip_path_ + "/pwm" + std::to_string(channel);

    // Export the PWM channel if not already exported.
    // 如果 pwm 通道未导出，则执行导出。
    if (access(pwm_path_.c_str(), F_OK) != 0) {
      FILE* fp = fopen((chip_path_ + "/export").c_str(), "w");
      if (fp) {
        (void)(fprintf(fp, "%d", channel));  // 导出通道
        (void)(fclose(fp));
        usleep(100000);  // Wait for sysfs to initialize.
      }
    }

    WriteSysfs(pwm_path_ + "/period", period_ns_);
    SetDutyCycle(duty_percent);
    Enable();
  }

  /**
   * @brief Destructor: disable PWM output.
   */
  ~PWM() { Disable(); }

  /**
   * @brief Set PWM output frequency.
   * @param hz Desired frequency in Hz.
   */
  void SetFrequency(uint32_t hz) {
    period_ns_ = static_cast<int>(1e9 / hz);
    WriteSysfs(pwm_path_ + "/period", period_ns_);
  }

  /**
   * @brief Set PWM duty cycle.
   * @param percent Duty cycle (0.0 ~ 1.0).
   */
  void SetDutyCycle(float percent) {
    int duty_ns = static_cast<int>(percent * static_cast<float>(period_ns_));
    WriteSysfs(pwm_path_ + "/duty_cycle", duty_ns);
  }

  /**
   * @brief Enable PWM output.
   */
  void Enable() { WriteSysfs(pwm_path_ + "/enable", 1); }

  /**
   * @brief Disable PWM output.
   */
  void Disable() { WriteSysfs(pwm_path_ + "/enable", 0); }

  /**
   * @brief Output a simple beep sound.
   * @param freq Frequency in Hz.
   * @param duration_ms Duration in milliseconds.
   */
  void Beep(uint32_t freq, uint32_t duration_ms) {
    SetFrequency(freq);
    SetDutyCycle(0.5);
    Enable();
    usleep(duration_ms * 1000);  // Delay in microseconds.
    Disable();
  }

  /**
   * @brief Play a musical note using MIDI pitch calculation.
   * @param note Note name (C, D, E, etc.).
   * @param octave Octave number (e.g., 4 for A4 = 440 Hz).
   * @param duration_ms Duration in milliseconds.
   */
  void PlayNote(NoteName note, uint32_t octave, uint32_t duration_ms) {
    float midi =
        static_cast<float>(note) + (static_cast<float>(octave) + 1.0f) * 12.0f;
    float freq = 440.0f * std::pow(2.0f, (midi - 69) / 12.0f);
    Beep(static_cast<uint32_t>(freq), duration_ms);
  }

  /**
   * @brief Configure alarm tone settings.
   * @param freq Alarm frequency in Hz.
   * @param duration_ms Tone duration in milliseconds.
   * @param delay_ms Delay after tone in milliseconds.
   */
  void SetAlarmConfig(uint32_t freq, uint32_t duration_ms, uint32_t delay_ms) {
    alarm_freq_ = freq;
    alarm_duration_ = duration_ms;
    alarm_delay_ = delay_ms;
  }

  /**
   * @brief Trigger alarm sound with delay.
   */
  void TriggerAlarm() {
    Beep(alarm_freq_, alarm_duration_);
    usleep(alarm_delay_ * 1000);
  }

 private:
  std::string chip_path_;  // PWM chip sysfs path
  std::string pwm_path_;   // PWM channel sysfs path
  int period_ns_;          // PWM period in nanoseconds
  int channel_;            // PWM channel number

  uint32_t alarm_freq_ = 1500;     // Default alarm frequency
  uint32_t alarm_duration_ = 300;  // Default alarm duration
  uint32_t alarm_delay_ = 300;     // Default delay after alarm

  /**
   * @brief Write integer value to sysfs file.
   * @param path Path to sysfs file.
   * @param value Value to write.
   * @return Write result or -1 on failure.
   */
  int WriteSysfs(const std::string& path, int value) const {
    FILE* fp = fopen(path.c_str(), "w");
    if (!fp) {
      return -1;
    }
    int result = fprintf(fp, "%d", value);
    (void)(fclose(fp));
    return result;
  }
};
